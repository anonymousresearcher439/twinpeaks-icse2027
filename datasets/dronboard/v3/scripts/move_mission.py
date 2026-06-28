#!/usr/bin/env python

import sys

from droneresponse_mathtools import Lla, mean_position


new_ceneter = Lla(
    float(sys.argv[1]),
    float(sys.argv[2]),
    float(sys.argv[3]),
)

mission_path = sys.argv[4]

import json
with open(mission_path, 'r') as f:
    mission = json.load(f)

# print(mission)
old_center = None

from typing import Dict, List
def is_pos(j: Dict)->bool:
    if "latitude" not in j or "longitude" not in j:
        return False
    
    return "altitude" in j or "relative_altitude" in j


def map_original_to_new(original: Lla) -> Lla:
    return Lla(
        new.latitude + (point.latitude - original.latitude),
        new.longitude + (point.longitude - original.longitude),
        new.altitude + (point.altitude - original.altitude)
    )

def scan_object(d: Dict, func):
    if is_pos(d):
        func(d)
    for k, v in d.items():
        if isinstance(v, dict):
            res = scan_object(v, func)
        if isinstance(v, list):
            for item in v:
                res = scan_object(item, func)

def make_lla(j: Dict) -> Lla:
    if "relative_altitude" in j:
        return Lla(j["latitude"], j["longitude"], j["relative_altitude"])
    return Lla(j["latitude"], j["longitude"], j["altitude"])

def find_all_pos(mission)-> List[Lla]:
    res = []
    def func(d: Dict):
        lla = make_lla(d)
        res.append(lla)
    scan_object(mission, func)
    return res

r = find_all_pos(mission)

old_center = mean_position(r)

def update_pos(j: Dict):
    if "relative_altitude" in j:
        original_lla = make_lla(j)
        n,e,_ = old_center.distance_ned(original_lla)
        new_lla = new_ceneter.move_ned(n,e,0)
        j["latitude"] = new_lla.latitude
        j["longitude"] = new_lla.longitude
        j["relative_altitude"] = j["relative_altitude"]
    if "altitude" in j:
        original_lla = make_lla(j)
        n,e,d = old_center.distance_ned(original_lla)
        new_lla = new_ceneter.move_ned(n,e,d)    

        j["latitude"] = new_lla.latitude
        j["longitude"] = new_lla.longitude
        j["altitude"] = new_lla.altitude


scan_object(mission, update_pos)

import pathlib
out_dir = sys.argv[5]

mission_path = pathlib.Path(mission_path)
# I want the filename from mission_path
out_file_name = mission_path.name
outpth = pathlib.Path(out_dir) / out_file_name

with open(outpth, 'w') as f:
    json.dump(mission, f, indent=4)


