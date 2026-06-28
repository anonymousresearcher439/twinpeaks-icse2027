import csv
import json
import sys
from typing import List

from droneresponse_mathtools import Lla
if len(sys.argv) != 2:
    print("Usage: python extract_airlease_requests.py <input.csv>")
    sys.exit(1)

filename = sys.argv[1]
filtered_requests = []
response_messages = []
first_location = None


def get_message(row):
    msg = json.loads(row["Message"])
    coordinates = [float(x) for x in msg["start_position"]]
    start_position = Lla(*coordinates)
    coordinates = [float(x) for x in msg["end_position"]]
    end_position = Lla(*coordinates)
    msg["start_position"] = start_position
    msg["end_position"] = end_position
    return msg

def transform_coordinates(message) -> None:
    """
    Transform coordinates from (lat, lon) to NED (very first location is origin).
    """
    start = message["start_position"]
    end = message["end_position"]
    start_ned = first_location.distance_ned(start)
    end_ned = first_location.distance_ned(end)
    message['start_ned'] = [round(x, 2) for x in start_ned]
    message['end_ned'] = [round(x, 2) for x in end_ned]


_next_number = 1
def next_nice_number():
    global _next_number
    result = _next_number
    _next_number += 1
    return result


_number_map = {}
def find_nice_number(request_number):
    """Given a request number, return a nice number.
    If we have seen this request number before, return the same nice number.
    Otherwise, return the next nice number.
    """
    if request_number in _number_map:
        return _number_map[request_number]
    else:
        nice_number = next_nice_number()
        _number_map[request_number] = nice_number
        return nice_number

def transform_request_number(message):
    """Given a message look at the request number and asign it nicer number 
    Starting with 1, then incrementing by 1 for each request.

    if we see the same request number again, we output the same number.
    
    """
    if "request_number" in message:
        request_number = message["request_number"]
        nice_number = find_nice_number(request_number)
        message["request_number"] = nice_number
    if "current_lease" in message and message["current_lease"] != 0:
        current_lease = message["current_lease"]
        nice_number = find_nice_number(current_lease)
        message["current_lease"] = nice_number
    return message


with open(filename, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        if first_location is None:
            # First row, set the first location
            first_location = get_message(row)['start_position']
        if row.get("Topic") == "airlease/request": #and float(row.get("Time")) > 1745422044.96052:
            try:
                msg = get_message(row)
                t = float(row.get("Delta T"))
                t = round(t, 3)
                msg["time"] = t
                filtered_requests.append(msg)
            except json.JSONDecodeError:
                print(f"Skipping row with invalid JSON: {row['Message']!r}")
                exit(1)
        if "drone/Polkadot/airlease/status" in row.get("Topic"):
            try:
                msg = json.loads(row["Message"])
                transform_request_number(msg)
                t = float(row.get("Delta T"))
                t = round(t, 3)
                msg["time"] = t
                response_messages.append(msg)
            except json.JSONDecodeError:
                print(f"Skipping row with invalid JSON: {row['Message']!r}")
                exit(1)

# Output: just to verify we got what we needed
for i, req in enumerate(filtered_requests):
    transform_coordinates(req)
    transform_request_number(req)
    important_data = {
        "time": req["time"],
        "start_ned": req["start_ned"],
        "end_ned": req["end_ned"],
        "request_number": req["request_number"],
        "current_lease": req.get("current_lease", 0),
    }
    print(f"Request {i+1}: {important_data}")
print(f"DONE: {len(filtered_requests)} requests found.")
print()
for response in response_messages:
    important_data = {
        'time': response.get('time'),
        'drone_id': response.get('drone_id'),
        'request_number': response.get('request_number'),
        'current_lease': response.get('current_lease'),
        'approved': response.get('approved'),
        'deadlock' : response.get('deadlock' ),
    }
    print(f"Response: {important_data}")





