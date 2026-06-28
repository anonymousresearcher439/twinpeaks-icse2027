from time import sleep
import unittest

from src.dr_onboard_autonomy.models.gimbal import GimbalAxes, GimbalStatus, Quaternion
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

        cls.heartbeat_sender: HeartbeatSender2 = HeartbeatSender2(
            communication_channel=MavlinkRouterTCP(),
            controller=cls.controller
        )
        cls.heartbeat_sender.start()

        # allow the gimbal to initialize mavlink mode before running tests
        sleep(5)

        cls.gimbal_gremsy: GimbalGremsy = GimbalGremsy(
            controller=cls.controller,
            gimbal_axes=GimbalAxes.PITCH | GimbalAxes.ROLL
        )

        # allow time for mavlink router to begin sending gimbal heartbeats to GimbalGremsy as the
        # heartbeats aren't routed to GimbalGremsy's port until mavlink router routes a message
        # from GimbalGremsy. This first message is sent at the end of GimbalGremsy's initializations
        sleep(2)


    @classmethod
    def tearDownClass(cls) -> None:
        cls.gimbal_gremsy.stop_communication()
        cls.heartbeat_sender.stop()   


    def test_gimbal_ready(self):
        self.assertEqual(self.gimbal_gremsy.get_gimbal_status().name, GimbalStatus.READY.name)


    def test_gimbal_sets_mount_configure(self):
        with self.gimbal_gremsy._do_mount_configure_status_lock:
            self.assertTrue(self.gimbal_gremsy._do_mount_configure_status_complete)


    def test_repeated_toggle_pitch_and_updates_current_attitude(self):
        runs = 0
        while runs < 2:
            desired_attitude = Quaternion((0, 0.7071068, 0, 0.7071068))
            self.gimbal_gremsy.set_attitude(desired_attitude)
            sleep(6)
            current_attitude = self.gimbal_gremsy.get_attitude()
            self.assertAlmostEqual(current_attitude[0], desired_attitude[0], 2)
            self.assertAlmostEqual(current_attitude[1], desired_attitude[1], 2)
            self.assertAlmostEqual(current_attitude[2], desired_attitude[2], 2)
            self.assertAlmostEqual(current_attitude[3], desired_attitude[3], 2)

            desired_attitude = Quaternion((0, -0.3836541, 0, 0.9234769))
            self.gimbal_gremsy.set_attitude(desired_attitude)
            sleep(6)
            current_attitude = self.gimbal_gremsy.get_attitude()
            self.assertAlmostEqual(current_attitude[0], desired_attitude[0], 2)
            self.assertAlmostEqual(current_attitude[1], desired_attitude[1], 2)
            self.assertAlmostEqual(current_attitude[2], desired_attitude[2], 2)
            self.assertAlmostEqual(current_attitude[3], desired_attitude[3], 2)

            runs += 1


    def test_set_neutral_attitude(self):
        sleep(3)
        self.gimbal_gremsy.set_neutral_attitude()
        sleep(6)
        current_attitude = self.gimbal_gremsy.get_attitude()
        self.assertAlmostEqual(current_attitude[0], 0, 2)
        self.assertAlmostEqual(current_attitude[1], 0, 2)
        self.assertAlmostEqual(current_attitude[2], 0, 2)
        self.assertAlmostEqual(current_attitude[3], 1, 2)


