import sys
import csv

from pathlib import Path
from string import Template
from unicodedata import name

import yaml
from droneresponse_mathtools import Lla, mean_position

SIMULATION_DIR = Path(__file__).parents[1] / "docker/simulations"
PX4_MODEL = "typhoon_h480"


class MyDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(MyDumper, self).increase_indent(flow, False)

from typing import List, Dict, TypedDict


def find_simulation_origin(drones: List[Dict])-> Lla:
    """
    Args:
        file_path: Path to the CSV file containing drone data.
    
    We expect the CSV file to have a headers with:
    - name: the name of the drone
    - latitude: the latitude of the drone
    - longitude: the longitude of the drone

    Returns the center point of the simulation 
    """
    
    # Convert latitude and longitude to Lla objects
    start_pos = [Lla(float(drone['latitude']), float(drone['longitude']), 0.0) for drone in drones]
    
    return mean_position(start_pos).to_lla()


def make_sitl_spec(drones: List[Dict], sim_origin: Lla) -> str:
    """
    Generates a SITL specification string that defines the starting positions for every drone.

    NOTE: In the `gazebo_sitl_multiple_run.sh` script, the `-s` option is used to specify the spawn location for each drone. This function generates the argument for the -s option.
    """

    # Create the starting conditions string
    starting_conditions = []
    for i, drone in enumerate(drones):
        system_id = i + 1  # Assuming sys_id starts from 1
        name = drone['name']
        home = Lla(float(drone['latitude']), float(drone['longitude']), sim_origin.altitude)
        n, e, _ = sim_origin.distance_ned(home)
        n = float(round(n,2))
        e = float(round(e,2))

        x = e
        y = n

        spec = f"{PX4_MODEL}:{system_id}:{x}:{y}"
        starting_conditions.append(spec)

    return ','.join(starting_conditions)

def main(file_path: Path, elevation: float):
    with open(file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        drones = [row for row in reader]
    # Validate the CSV file structure
    assert len(drones) > 0, "CSV file must have at least one drone entry"
    assert "name" in drones[0], "CSV file must have a 'name' column"
    assert 'latitude' in drones[0], "CSV file must have a 'latitude' column"
    assert 'longitude' in drones[0], "CSV file must have a 'longitude' column"
    # make sure every name is a string and every latitude and longitude is a float
    for drone in drones:
        assert isinstance(drone['name'], str), "Drone name must be a string"
        assert isinstance(float(drone['latitude']), float), "Drone latitude must be a float"
        assert isinstance(float(drone['longitude']), float), "Drone longitude must be a float"
        # also make sure the latitude goes from -90 to 90 and longitude from -180 to 180
        assert -90 <= float(drone['latitude']) <= 90, "Drone latitude must be between -90 and 90"
        assert -180 <= float(drone['longitude']) <= 180, "Drone longitude must be between -180 and 180"
    
    sim_origin = find_simulation_origin(drones)
    sim_origin = Lla(sim_origin.latitude, sim_origin.longitude, elevation)  # Set the elevation
    starting_conditions = make_sitl_spec(drones, sim_origin)
    env_data = [
        # f"NUM_DRONES={len(drones)}",
        f"PX4_HOME_LAT={sim_origin.latitude}",
        f"PX4_HOME_LON={sim_origin.longitude}",
        f"PX4_HOME_ALT={sim_origin.altitude}",
        "HEADLESS=1",
        "NO_PXH=1",
        # f"SIM_STARTING_CONDITIONS={starting_conditions}",
    ]

    for data in env_data:
        print(data)
    


    template = Template((SIMULATION_DIR / "multi/drone.yaml.template").read_text())
    config_template = Template((SIMULATION_DIR / "multi/config.toml.template").read_text())


    # ok if we made it this far, we can proceed
    for p in (SIMULATION_DIR / Path("multi/")).glob("*.yaml"):
        p.unlink(True)
    
    depends_on = []
    includes = []

    for i, drone in enumerate(drones):
        drone_name = drone['name']
        local_mqtt_port = 1884 + i

        file_content = template.substitute(
                n=i,
                sys_id=i + 1,
                local_mqtt_port=local_mqtt_port,
                drone_name=drone_name,
                config_file=f"config_{i}.toml",
        )
        output_path = SIMULATION_DIR / f"multi/{i}.yaml"
        output_path.write_text(file_content)

        depends_on.append(f"mavros_{i}")
        includes.append(f"./multi/{i}.yaml")

        config_content = config_template.substitute(
            name = drone_name,
            system_id = i + 1, # this is the MAVLink system ID
            drone_registration_id = drone.get("drone_registration_id", "UNKNOWN DRONE REGISTRATION ID"),
            pilot_id = drone.get("pilot_id", "UNKNOWN PILOT ID"),
            gcs_host = drone.get("gcs_host", "host.docker.internal"),
            local_mqtt_host = f"mqtt_local_{i}",
            fcu = "px4",
            owner_id = drone.get("owner_id", "UNKNOWN OWNER ID"),
            model_name = drone.get("model_name", "UNKNOWN MODEL NAME"),
        )
        config_path = SIMULATION_DIR / f"multi/config_{i}.toml"
        config_path.write_text(config_content)

    multi = yaml.safe_load((SIMULATION_DIR / "multi.yaml.template").read_text())
    multi["services"]["px4"]["depends_on"] = depends_on
    multi["include"] = includes
    multi["services"]["px4"]["command"] = f'start_simulated_scenario.sh "{starting_conditions}"'
    multi["services"]["px4"]["environment"] = env_data
    (SIMULATION_DIR / "multi.yaml").write_text(
        yaml.dump(multi, indent=2, Dumper=MyDumper, default_flow_style=False)
    )



if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <csv_file_path> <elevation>")
        sys.exit(1)
    input_file_path = Path(sys.argv[1])
    elevation = float(sys.argv[2])
    main(input_file_path, elevation)