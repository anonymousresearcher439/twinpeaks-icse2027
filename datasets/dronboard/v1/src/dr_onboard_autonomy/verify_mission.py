from unittest.mock import NonCallableMock, patch

from dr_onboard_autonomy.briar_helpers import BriarLla, briarlla_from_lat_lon_alt
from dr_onboard_autonomy.message_senders import AbstractMessageSender, ReusableMessageSenders
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.state_machine import verify_mission, build_default_args


class MockReadPosition:
    def __init__(self, return_channel, **args):
        pos = briarlla_from_lat_lon_alt(0, 0, 0, is_amsl=True)
        return_channel.put(pos)
    
    def execute(self, _):
        return "succeeded"

def mock_args():
    uav_name="TEST_UAV"

    mock_drone = NonCallableMock(spec=MAVROSDrone)
    mock_drone.uav_name = uav_name
    mock_drone.system_id = 1
    mock_drone.fc_component_id = 100

    mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
    mock_reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)
    mock_reusable_msg_senders.configure_mock(**{"find.return_value": mock_msg_sender})
    
    mock_mqtt = NonCallableMock(spec=MQTTClient)
    with patch('dr_onboard_autonomy.state_machine.ReadPosition', new=MockReadPosition) as M:
        args = build_default_args(mock_drone, mock_reusable_msg_senders, mock_mqtt, mock_mqtt)
    return args

mission_args = mock_args()


def check_mission(json_str):
    result = verify_mission(json_str, mission_args)
    if type(result) is bool and result:
        print("MISSION OK")
    else:
        print(result)

import sys

file_path=sys.argv[1]
with open(file_path) as mission_file:
    j = mission_file.read()
    print(j)
    check_mission(j)