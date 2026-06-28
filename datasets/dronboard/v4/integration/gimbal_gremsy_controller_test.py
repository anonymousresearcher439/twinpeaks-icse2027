from time import sleep
import unittest

from src.dr_onboard_autonomy.models import Quaternion
from src.dr_onboard_autonomy.models.gimbal import GimbalAxes, GimbalStatus
from src.dr_onboard_autonomy.gimbal.gimbal_gremsy import GimbalGremsy
from src.dr_onboard_autonomy.mavlink import (
    ComponentId,
    HeartbeatSender2,
    MavlinkRouterTCP,
    MavlinkNode,
    SystemId
)


class TestGimbalGremsy(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller: MavlinkNode = MavlinkNode(
            component_id=ComponentId(25),
            system_id=SystemId(1)
        )

        cls.gimbal_gremsy: GimbalGremsy = GimbalGremsy(
            controller=cls.controller,
            gimbal_axes=GimbalAxes.PITCH | GimbalAxes.ROLL
        )

        # allow time for mavlink router to begin sending gimbal heartbeats to GimbalGremsy as the
        # heartbeats aren't routed to GimbalGremsy's port until mavlink router routes a message
        # from GimbalGremsy. This first message is sent at the end of GimbalGremsy's initializations
        sleep(5)


    @classmethod
    def tearDownClass(cls) -> None:
        cls.gimbal_gremsy.stop_communication()


    def test_gimbal_ready(self):
        self.assertEqual(self.gimbal_gremsy.get_gimbal_status().name, GimbalStatus.READY.name)


    def test_repeated_toggle_pitch_and_updates_current_attitude(self):
        runs = 0
        while runs < 2:
            # pitch +90 degrees to look UP 
            desired_attitude = Quaternion(0, 0.7071068, 0, 0.7071068)
            self.gimbal_gremsy.set_attitude(desired_attitude)
            sleep(3)
            current_attitude = self.gimbal_gremsy.get_attitude()
            self.assertAlmostEqual(current_attitude.x, desired_attitude.x, 2)
            self.assertAlmostEqual(current_attitude.y, desired_attitude.y, 2)
            self.assertAlmostEqual(current_attitude.z, desired_attitude.z, 2)
            self.assertAlmostEqual(current_attitude.w, desired_attitude.w, 2)

            # roll 10 degrees
            desired_attitude = Quaternion(0.08711167063, -0.0, 0.0, 0.9961985529)
            self.gimbal_gremsy.set_attitude(desired_attitude)
            sleep(3)
            current_attitude = self.gimbal_gremsy.get_attitude()
            self.assertAlmostEqual(current_attitude.x, desired_attitude.x, 2)
            self.assertAlmostEqual(current_attitude.y, desired_attitude.y, 2)
            self.assertAlmostEqual(current_attitude.z, desired_attitude.z, 2)
            self.assertAlmostEqual(current_attitude.w, desired_attitude.w, 2)

            # pitch -45 degrees
            desired_attitude = Quaternion(0, -0.3836541, 0, 0.9234769)
            self.gimbal_gremsy.set_attitude(desired_attitude)
            sleep(3)
            current_attitude = self.gimbal_gremsy.get_attitude()
            self.assertAlmostEqual(current_attitude.x, desired_attitude.x, 2)
            self.assertAlmostEqual(current_attitude.y, desired_attitude.y, 2)
            self.assertAlmostEqual(current_attitude.z, desired_attitude.z, 2)
            self.assertAlmostEqual(current_attitude.w, desired_attitude.w, 2)

            runs += 1


    def test_set_neutral_attitude(self):
        self.gimbal_gremsy.set_neutral_attitude()
        sleep(3)
        current_attitude = self.gimbal_gremsy.get_attitude()
        self.assertAlmostEqual(current_attitude.x, 0, 2)
        self.assertAlmostEqual(current_attitude.y, 0, 2)
        self.assertAlmostEqual(current_attitude.z, 0, 2)
        self.assertAlmostEqual(current_attitude.w, 1, 2)


