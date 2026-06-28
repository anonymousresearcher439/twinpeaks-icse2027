import unittest
from dr_onboard_autonomy import state_factory
from dr_onboard_autonomy.states import * # import all the states...


class TestStateRegistrationData(unittest.TestCase):
    """
    This class tests the state registration data.
    """
    
    def test_that_all_default_transitions_are_correct(self):
        """
        This tests the state registration data. As part of adding the state registration decorator, 
        we need to make sure that our states are registered with the correct transitions. 
        Updating the states to use the decorator required manually specifying the default transition 
        dictionary into the decorator. This test makes sure there were no mistakes.

        NOTE: If someone changes the default transitions for one of these classes in the future, 
        it would be best to not test its transitions here, but to instead create a unit test 
        that's specific to the class. So if you change the Arm state so that it has different 
        default transitions, I recommend updating this test so that it doesn't look at the arm state. 
        And instead you should add a new test to the suite of Arm tests, and this is where the 
        default transitions should be verified. This test is designed to ensure that we didn't 
        break any legacy code. This data was all specified earlier, and this was the most practical 
        way to test the change.
        """
        all_states = [
            "AbortHover",
            "Arm",
            "BetterCircle",
            "BetterHover",
            "BetterPath",
            "BriarCircle",
            "BriarHover",
            "BriarTravel",
            "BriarWaypoint",
            "BriarWaypoint2",
            "BriarWaypoint3",
            "CircleVisionTarget",
            "CircleTargetPosition",
            "Disarm",
            "FlyWaypoints",
            "GimbalTestFixedEuler",
            "GimbalTestFixedQuaternion",
            "GimbalTestStarePoint",
            "HeartbeatHover",
            "Hover",
            "Failsafe",
            "Land",
            "PhasedCircle",
            "ReadMessages",
            "ReadMessagesAirborne",
            "Rtl",
            "Takeoff",
            "Follow_with_cvTracking",
        ]

        all_expected_transitions = {
            "AbortHover": {
                "error": "failure",
                "failsafe": "Failsafe"
            },
            "Arm": {
                "error": "failure",
            },
            "BetterCircle": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "BetterHover": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "BetterPath": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "BriarHover": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge" 
            },
            "BriarTravel": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"   
            },
            "BriarWaypoint": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "BriarWaypoint2": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "BriarWaypoint3": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "BriarCircle": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "CircleTargetPosition": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "CircleVisionTarget": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "Disarm": {
                "error": "failure",
            },
            "FlyWaypoints": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "GimbalTestStarePoint": {
                "error": "failure",
            },
            "GimbalTestFixedQuaternion": {
                "error": "failure",
            },
            "GimbalTestFixedEuler": {
                "error": "failure",
            },
            "HeartbeatHover": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "Hover": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "Land": {
                "error": "failure",
                "failsafe": "Failsafe",
                "rtl": "Rtl"
            },
            "ReadMessages": {
                "error": "failure",
            },
            "ReadMessagesAirborne": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "Takeoff": {
                "error": "failure",
                "failed_takeoff": "Land",
                "failsafe": "Failsafe",
                "rtl": "Rtl"
            },
            "Failsafe": {
                "succeeded": "failure",
                "error": "failure",
            },
            "OnGround": {
                "error": "failure",
            },
            "PhasedCircle": {
                "error": "failure",
                "failsafe": "Failsafe",
                "abort": "AbortHover",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            },
            "PositionDrone": {
                "error": "failure",
            },
            "PossibleVictimDetected": {
                "error": "failure",
            },
            "Rtl": {
                "error": "failure",
                "failsafe": "Failsafe"
            },
            "Searching": {
                "error": "failure",
            },
            "Standby": {
                "error": "failure",
            },
            "Tracking": {
                "error": "failure",
            },
            "VictimFound": {
                "error": "failure",
            },
            "Follow_with_cvTracking":{
                "abort": "AbortHover",
                "error": "failure",
                "failsafe": "Failsafe",
                "rtl": "Rtl",
                "low_battery": "ReturnToRecharge"
            }
        }
        print(state_factory._registry_data.dump_debug_table())
        for state in all_states:
            # get the expected transitions for this state
            tx = all_expected_transitions[state]
            # get the actual transitions for this state
            _, actual_tx = state_factory._registry_data.find(state)
            # make sure tx == actual_tx
            self.assertDictEqual(tx, actual_tx, f"The default transitions for state: {state} are not correct")

if __name__ == '__main__':
    unittest.main()