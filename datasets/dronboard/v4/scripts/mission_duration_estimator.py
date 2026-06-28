# This script calculates the time it takes to fly between a series of waypoints
# and provides an estimate of the total time it will take to fly the entire mission
# The waypoints are read from a JSON file
# The speed limit is provided as an argument to the script (default is 6.0 m/s)
# Usage:
# python3 mission_duration_estimator.py <file path to JSON file> <speed limit>

# Example File:
"""
[
  {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 124
  },
  {
    "latitude": 37.775107217395664,
    "longitude": -122.41913893812763,
    "altitude": 124.00008300962553
  },
  {
    "latitude": 37.7753144345812,
    "longitude": -122.41879842169152,
    "altitude": 124.00019506481654
  },
  {
    "latitude": 37.77552165175941,
    "longitude": -122.41845790430503,
    "altitude": 124.00030712001285
  },
  {
    "latitude": 37.77572886893031,
    "longitude": -122.4181173859681,
    "altitude": 124.00041917521447
  },
  {
    "latitude": 37.7759360860939,
    "longitude": -122.41777686668075,
    "altitude": 124.00053123042143
  },
  {
    "latitude": 37.77614330325019,
    "longitude": -122.41743634644297,
    "altitude": 124.00064328421566
  },
  {
    "latitude": 37.776350520399156,
    "longitude": -122.41709582525473,
    "altitude": 124.00075533801522
  },
  {
    "latitude": 37.77655773754082,
    "longitude": -122.41675530311603,
    "altitude": 124.00086739323808
  },
  {
    "latitude": 37.77676495467516,
    "longitude": -122.41641478002686,
    "altitude": 124.00097944704831
  },
  {
    "latitude": 37.77697217180219,
    "longitude": -122.41607425598718,
    "altitude": 124.00109150228181
  },
  {
    "latitude": 37.77717938892191,
    "longitude": -122.41573373099705,
    "altitude": 124.00120355752065
  },
  {
    "latitude": 37.77738660603431,
    "longitude": -122.41539320505642,
    "altitude": 124.00131561134677
  }
]
"""
from droneresponse_mathtools import Lla
from ruckig import InputParameter, Result, Ruckig, Trajectory
import json

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.models.drone import CopterParameters

SPEED_LIMIT = 6.0  # m/s

constraints = CopterParameters(
    horizontal_acceleration_limit=3.0,
    upward_acceleration_limit=4.0,
    downward_acceleration_limit=3.0,
    jerk_limit=4.0,

    # the following are not used:
    takeoff_altitude=10.0,
    maximum_yaw_rate=0.8,
)

# read a list of LLA coordinates from a JSON file
def read_coordinates(filename):
    """Example JSON
    [
        {"latitude": 37.7749, "longitude": -122.4194, "altitude": 0},
        {"latitude": 37.7749, "longitude": -122.4194, "altitude": 0},
        {"latitude": 37.7749, "longitude": -122.4194, "altitude": 0}
    ]
    """
    with open(filename, 'r') as file:
        result = []
        json_data = json.load(file)
        for location in json_data:
            result.append(Lla(location["latitude"], location["longitude"], location["altitude"]))
        return result

def make_trajectory(start, end, speed_limit):
   # def _make_trajectory(self, destination: LlaPosition, speed_limit: float, existing_context: Optional[Context]) -> Context:
    """
    This function creates a trajectory Context. This struct-like object includes a trajectory calculator from the Ruckig library.

    Our trajectories follow a certain pattern:

    They're always specified in NED coordinates.

    If we're not given an existing context, we start from the origin (0,0,0) and move to the destination.
    If we're given an existing context, we start from the existing context's origin and move to the destination.

    The starting position is always (0,0,0)
    The final position is the NED coordinates of the final position.


    """
    global constraints
    # TODO assert we have all the data we need to calculate a trajectory
    # We're going to create a Context object

    position_initial = BriarLla.from_lla(start, is_amsl=True)
    
    position_final = BriarLla.from_lla(end, is_amsl=True)


    trajectory_calculator: Ruckig.Trajectory
    time_final: float
    time_duration: float

    inp: InputParameter = InputParameter(3)

    inp.current_position = [0.0, 0.0, 0.0]
    inp.current_velocity = [0.0, 0.0, 0.0]
    inp.current_acceleration = [0.0, 0.0, 0.0]

    final_position_ned = position_initial.ellipsoid.lla.distance_ned(position_final.ellipsoid.lla)
    inp.target_position = [float(x) for x in final_position_ned]
    inp.target_velocity = [0.0, 0.0, 0.0]
    inp.target_acceleration = [0.0, 0.0, 0.0]

    inp.max_velocity = [speed_limit, speed_limit, speed_limit]
    
    max_horizontal_acceleration = constraints.horizontal_acceleration_limit
    max_down_acceleration = constraints.downward_acceleration_limit
    max_up_acceleration = constraints.upward_acceleration_limit
    inp.max_acceleration = [
        max_horizontal_acceleration,
        max_horizontal_acceleration,
        max_down_acceleration
    ]
    inp.min_acceleration = [
        -1.0 * max_horizontal_acceleration,
        -1.0 * max_horizontal_acceleration,
        -1.0 * max_up_acceleration
    ]
    max_jerk = constraints.jerk_limit
    inp.max_jerk = [
        max_jerk,
        max_jerk,
        max_jerk
    ]

    otg = Ruckig(3)
    trajectory = Trajectory(3)

    # Calculate the trajectory in an offline manner
    calc_result = otg.calculate(inp, trajectory)
    if calc_result == Result.ErrorInvalidInput:
        raise Exception('Invalid input!')

    return trajectory

# read the file path from the command line
# also read the speed limit from the command line
# arguments are the file path and the speed limit
# if a speed limit isn't provided, use the default speed limit
import sys
if len(sys.argv) < 2:
    print("Usage: python3 trajectory.py <file path to JSON file> <speed limit>")
    sys.exit(1)

coordinates = read_coordinates(sys.argv[1])
print(f"found {len(coordinates)} coordinates in {sys.argv[1]}")

if len(sys.argv) > 2:
    SPEED_LIMIT = float(sys.argv[2])
print(f"Speed limit: {SPEED_LIMIT} m/s")

print(f"Calculating the time it takes to fly this mission...")
# I need to calculate the trajectory between each pair of coordinates
# for example coordinates[0] to coordinates[1], coordinates[1] to coordinates[2], etc.
total_time = 0.0
for i in range(len(coordinates) - 1):
    start = coordinates[i]
    end = coordinates[i + 1]
    trajectory = make_trajectory(start, end, SPEED_LIMIT)
    total_time = total_time + trajectory.duration + 2.0

print(f"Total time: {str(total_time)} seconds")


