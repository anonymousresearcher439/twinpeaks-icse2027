import sys
import unittest

from dr_onboard_autonomy.briar_helpers import BriarLla, convert_LlaDict_to_tuple
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from unittest.mock import (
    MagicMock,
    NonCallableMagicMock, 
    patch
)


def mock_drone():
        drone = NonCallableMagicMock(spec=MAVROSDrone)
        return drone    


class TestPhasedCircle(unittest.TestCase):
    def setUp(self):
        self.mock_num_arcs = 3
        self.mock_pause_time = 15.0
        self.mock_center_position = {
            'latitude': 41.70573086523541, 
            'longitude': -86.24421841999177, 
            'altitude': 218.566
        }
        self.mock_stare_position = {
            'latitude': 41.70573086523541, 
            'longitude': -86.24421841999177, 
            'altitude': 150.0
        }
        self.mock_total_sweep = 90
        self.mock_pitch = 20.0
        self.mock_distance = 200.0
        self.mock_starting_angle = 270.0
        self.mock_reusable_message_senders = MagicMock(spec=ReusableMessageSenders)
        self.mock_mqtt_client = MagicMock(spec=MQTTClient)
        self.mock_userdata = NonCallableMagicMock()

    '''
    use patch.object with sys module ref due to PhasedCircle import in 
    dr_onboard_autonomy.states.__init__.py
    https://stackoverflow.com/questions/32026987/mock-patch-with-init-py
    '''
    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BriarHover")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BetterHover")
    def test_success(self, mock_better_hover, mock_briar_hover, mock_briar_circle, 
    mock_heartbeat_status_handler):
        from dr_onboard_autonomy.states.PhasedCircle import PhasedCircle

        mock_briar_circle.return_value.execute.return_value = "succeeded_circle"
        mock_briar_circle.return_value.stare_position = BriarLla(self.mock_stare_position)
        mock_briar_hover.return_value.execute.return_value = "succeeded_hover"
        mock_better_hover.return_value.execute.return_value = "succeeded_hover"

        
        phased_circle = PhasedCircle(
            pitch=self.mock_pitch,
            distance=self.mock_distance,
            starting_angle=self.mock_starting_angle,
            center_position=self.mock_center_position,
            stare_position=self.mock_stare_position,
            total_sweep_angle=self.mock_total_sweep,
            number_arcs=self.mock_num_arcs,
            pause_time=self.mock_pause_time,
            reusable_message_senders = self.mock_reusable_message_senders,
            mqtt_client = self.mock_mqtt_client,
            drone=mock_drone(),
            data={},
        )

        outcome = phased_circle.execute(self.mock_userdata)

        self.assertEqual(outcome, "succeeded_circle")
        '''
        what is userdata used for? see it passed into functions, but haven't seen it utilized
        '''

        briar_hover_arg = mock_briar_hover.call_args.kwargs
        self.assertEqual(briar_hover_arg["hover_time"], self.mock_pause_time)
        self.assertEqual(briar_hover_arg["stare_position"], self.mock_stare_position)

        briar_circle_arg = mock_briar_circle.call_args.kwargs
        self.assertEqual(briar_circle_arg["center_position"], self.mock_center_position)
        self.assertEqual(briar_circle_arg["stare_position"], self.mock_stare_position)
        self.assertEqual(briar_circle_arg["sweep_angle"], self.mock_total_sweep / self.mock_num_arcs)

        better_hover_arg = mock_better_hover.call_args.kwargs
        self.assertEqual(better_hover_arg["pitch"], self.mock_pitch)
        self.assertEqual(better_hover_arg["stare_position"], self.mock_center_position)
        self.assertEqual(better_hover_arg["distance"], self.mock_distance)
        self.assertEqual(better_hover_arg["starting_angle"], self.mock_starting_angle)
        self.assertEqual(better_hover_arg["hover_time"], self.mock_pause_time)

        self.assertEqual(mock_briar_hover.call_count, self.mock_num_arcs)
        self.assertEqual(mock_briar_circle.call_count, self.mock_num_arcs)

    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BetterHover")
    def test_failure_better_hover(self, mock_better_hover, mock_heartbeat_status_handler):
        from dr_onboard_autonomy.states.PhasedCircle import PhasedCircle

        mock_failure_outcome = "some failure outcome"
        
        mock_better_hover.return_value.execute.return_value = mock_failure_outcome

        phased_circle = PhasedCircle(
            pitch=self.mock_pitch,
            distance=self.mock_distance,
            starting_angle=self.mock_starting_angle,
            center_position=self.mock_center_position,
            stare_position=self.mock_stare_position,
            total_sweep_angle=self.mock_total_sweep,
            number_arcs=self.mock_num_arcs,
            pause_time=self.mock_pause_time,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=mock_drone(),
            data={},
        )

        outcome = phased_circle.execute(self.mock_userdata)

        self.assertEqual(outcome, mock_failure_outcome)

    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BriarHover")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BetterHover")
    def test_failure_briar_circle(self, mock_better_hover, mock_briar_hover, mock_briar_circle,
    mock_heartbeat_status_handler):
        from dr_onboard_autonomy.states.PhasedCircle import PhasedCircle

        mock_failure_outcome = "some failure outcome"
        
        mock_briar_circle.return_value.execute.return_value = mock_failure_outcome
        mock_briar_circle.return_value.stare_position_amsl_tup = convert_LlaDict_to_tuple(
            self.mock_stare_position
        )
        mock_better_hover.return_value.execute.return_value = "succeeded_hover"

        phased_circle = PhasedCircle(
            pitch=self.mock_pitch,
            distance=self.mock_distance,
            starting_angle=self.mock_starting_angle,
            center_position=self.mock_center_position,
            stare_position=self.mock_stare_position,
            total_sweep_angle=self.mock_total_sweep,
            number_arcs=self.mock_num_arcs,
            pause_time=self.mock_pause_time,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=mock_drone(),
            data={},
        )

        outcome = phased_circle.execute(self.mock_userdata)

        self.assertEqual(outcome, mock_failure_outcome)

    @patch.object(sys.modules["dr_onboard_autonomy.states.BaseState"], "HeartbeatStatusHandler")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BriarHover")
    @patch.object(sys.modules["dr_onboard_autonomy.states.PhasedCircle"], "BetterHover")
    def test_failure_briar_hover(self, mock_better_hover, mock_briar_hover, mock_briar_circle,
    mock_heartbeat_status_handler):
        from dr_onboard_autonomy.states.PhasedCircle import PhasedCircle

        mock_failure_outcome = "some failure outcome"
        
        mock_briar_circle.return_value.execute.return_value = "succeeded_circle"
        mock_briar_circle.return_value.stare_position_amsl_tup = convert_LlaDict_to_tuple(
            self.mock_stare_position
        )
        mock_better_hover.return_value.execute.return_value = "succeeded_hover"
        mock_briar_hover.return_value.execute.return_value = mock_failure_outcome

        phased_circle = PhasedCircle(
            pitch=self.mock_pitch,
            distance=self.mock_distance,
            starting_angle=self.mock_starting_angle,
            center_position=self.mock_center_position,
            stare_position=self.mock_stare_position,
            total_sweep_angle=self.mock_total_sweep,
            number_arcs=self.mock_num_arcs,
            pause_time=self.mock_pause_time,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=mock_drone(),
            data={},
        )

        outcome = phased_circle.execute(self.mock_userdata)

        self.assertEqual(outcome, mock_failure_outcome)
        


        
