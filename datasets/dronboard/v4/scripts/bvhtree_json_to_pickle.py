from dr_onboard_autonomy.gimbal.geolocation_boundingvols import BVHTree
from dr_onboard_autonomy.gimbal.geolocator import GeoLocator
from droneresponse_mathtools import Lla
import json
from pathlib import Path
import numpy as np
import joblib
import argparse


def get_center_lla(file: Path):
    """Get LLA roughly in center of elevation map
    """
    data = np.asarray(json.loads(Path(file).read_text()))
    lat_mean, lon_mean, _ = data.mean(axis=0)

    # Get index closest to mean lat and mean lon
    lat_err = (np.abs(data[:,0] - lat_mean))
    lon_err = (np.abs(data[:,1] - lon_mean))
    idx = (lat_err+lon_err).argmin()
    return data[idx].tolist()


def geolocate_test(file: Path):
    # Get lla centerpoint from json file
    lla_ground = get_center_lla(file)
    lla_drone = (lla_ground[0], lla_ground[1], lla_ground[2] + 20)

    # Geolocate
    folder = file.parent
    gl = GeoLocator(Lla(*lla_drone))
    gl.load_tree(folder, folder)

    fov_h = 74  # degrees
    image_res = (1920, 1080)
    target_coords = (960, 810) # pixel of target
    ground_alt = 165

    qs =  [
        (0, 0.7071068, 0, 0.7071068), # camera looks down down and to the west
        (0.056115267, -0.0154703723, 0.9608545, 0.27086953), # camera looks down and to the NW at ~10 degrees
        (0, 0.3826834, 0, 0.9238795), # camera looks 45 degrees down and to the east
        (-0.270598, 0.2705981, 0.6532814, 0.6532816), # camera looks 45 degrees down and to the north
    ]

    # Double check we get intersections found using BVH Tree
    for quaternion_gimbal in qs:
        try:
            location = gl.geolocate_object_from_camera(
                fov_h=fov_h,
                lla=lla_drone,
                image_res=image_res,
                target_coords=target_coords,
                quaternion_gimbal=quaternion_gimbal,
                ground_alt=ground_alt,
            )
            print(location)
        except Exception as e:
            print(f"Could not geolocate: {e}")



def create_pickle(filepath: Path):
    bvhtree = BVHTree.from_json(filepath)
    output_name = f"{filepath.stem}.joblib.gz"
    output_path = filepath.parent / output_name

    joblib.dump(bvhtree, output_path, compress=3, protocol=5)
    print(f'Pickled BVHTree created: {output_path}')


def parse_args():
    """
    Usage without geolocation testing:
    > python scripts/bvhtree_json_to_pickle.py <folder>

    Usage with geolocation testing:
    > python scripts/bvhtree_json_to_pickle.py <folder> --test_geolocate
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=str, help='Path to folder with json elevation data')
    parser.add_argument('--test_geolocate', action='store_true', default=False, help='Whether to test the geolocation using the pickled BVHtree')
    args = parser.parse_args()
    print(f'Folder for processing: {args.folder} \nGeolocation tests: {args.test_geolocate}')

    return Path(args.folder), args.test_geolocate


if __name__ == '__main__':
    folder, geolocate = parse_args()

    # Get all files in folder with .json suffix
    for filepath in folder.glob('*.json'):
        print(f"Processing {filepath}")
        create_pickle(filepath)

        if geolocate:
            geolocate_test(filepath)

