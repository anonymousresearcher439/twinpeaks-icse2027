#!/usr/bin/env python3

DESCRIPTION = """Verify that missions will initalize OK.

This script verifies the integrity of a missions. It provides a way to check that the missions are "OK" in a specific sense.

This program reads one or more mission files, which store JSON strings. These JSON strings describe how to build an event-driven state machine for a drone. 

The verification process checks that:
1. The mission file is valid JSON and can be parsed without errors.
2. The mission references types of states that exist in the system.
3. The mission provides all the required arguments for initializing these states.
4. all transition targets are states that are in the mission.
"""

import os
import sys
import argparse
import glob

from unittest.mock import patch
from dr_onboard_autonomy.mission_helper import MissionBuilder
from test import mock_types

from smach import InvalidTransitionError

parser = argparse.ArgumentParser(
    description=DESCRIPTION,
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
It's important to note the limitations of this script. While it verifies the structural integrity of the mission, it does not evaluate the logic or safety of the mission. For example, a mission that instructs the drone to fly into the ground would pass this verification, as it is structurally sound but logically flawed. Therefore, while this script helps catch certain types of errors, it is not going to find every possible problem.
"""
)

# Add the arguments
parser.add_argument('paths', metavar='PATH', type=str, nargs='*', 
                    default=['./missions'], 
                    help="The file paths to verify. Can be a mix of files and directories. In case you give a directory this finds every JSON file in that directory, and verifies all of them (but it doesn't search recursively).  (default: ['./missions'])")

# Parse the arguments
args = parser.parse_args()


def make_mission_builder():
    """Create a mission builder with mock arguments."""
    args = mock_types.mock_mission_builder_args()
    del args["state_factory"]

    mission_builder = MissionBuilder(**args)
    mission_builder.setup()
    return mission_builder


def verify_mission(json_str, mission_builder):
    """Verify that a mission is valid."""
    with patch('dr_onboard_autonomy.mission_helper.rospy') as mock_ros:
        mock_ros.logfatal = lambda *args, **kwargs: print("[ FATAL ] :", *args, **kwargs)
        mock_ros.logerr   = lambda *args, **kwargs: print("[ ERROR ] :", *args, **kwargs)
        mock_ros.logwarn  = lambda *args, **kwargs: print("[ WARN ] :", *args, **kwargs)
        mock_ros.loginfo  = lambda *args, **kwargs: print("[ INFO ] :", *args, **kwargs)
        mock_ros.logdebug = lambda *args, **kwargs: print("[ DEBUG ] :", *args, **kwargs)

        mission = mission_builder.build_mission(json_str)
        mission.check_consistency()
    return True


def get_files_to_verify():
    # Create the parser
    # List to store files to verify
    files_to_verify = []
    
    # Iterate over command line arguments
    for path in args.paths:
        # If it's a file, add it to the list
        if os.path.isfile(path):
            files_to_verify.append(path)
        # If it's a directory, find all JSON files in it and add them to the list
        elif os.path.isdir(path):
            files_to_verify.extend(glob.glob(f'{path}/**/*.json', recursive=True))
    files_to_verify.sort()
    return files_to_verify


def main():
    """Verify that missions will initalize OK."""
    # Create a mission builder with mock arguments
    mission_builder = make_mission_builder()

    # Get the files to verify
    files_to_verify = get_files_to_verify()

    # Iterate over the files to verify
    for file_path in files_to_verify:
        # Read the file
        print(f"Reading: {file_path}", file=sys.stderr, flush=True)
        with open(file_path, 'r') as f:
            json_str = f.read()
        
        # Verify the mission
        try:
            verify_mission(json_str, mission_builder)
        except Exception as err:
            print(f"Error: '{file_path}' failed verification. {err}", file=sys.stderr)
            err.args = (*err.args, f"File path: {file_path}")
            raise

        # Print the result
        print(f"Verified: {file_path}", file=sys.stderr, flush=True)
    
    print("done checking missions", file=sys.stderr, flush=True)
    print("list of missions checked:", file=sys.stderr, flush=True)
    for file_path in files_to_verify:
        print(f"    {file_path}", file=sys.stderr, flush=True)
    print("All missions OK.", file=sys.stderr, flush=True)

if __name__ == "__main__":
    main()