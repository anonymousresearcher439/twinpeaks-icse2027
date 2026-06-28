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
            "HumanControl",
            "Land",
            "PhasedCircle",
            "Preflight",
            "ReadMessages",
            "ReadMessagesAirborne",
            "Rtl",
            "UnstableTakeoff",
            "Takeoff",
            "Follow_with_cvTracking",
        ]

        all_expected_transitions = {
            "AbortHover": {
                "error": "failure",
                "human_control": "HumanControl"
            },
            "Arm": {
                "error": "failure",
            },
            "BetterCircle": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "BetterHover": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "BetterPath": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "BriarHover": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl" 
            },
            "BriarTravel": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"     
            },
            "BriarWaypoint": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "BriarWaypoint2": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"  
            },
            "BriarWaypoint3": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"  
            },
            "BriarCircle": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "CircleTargetPosition": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "CircleVisionTarget": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "Disarm": {
                "error": "failure",
            },
            "FlyWaypoints": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
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
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "Hover": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "Land": {
                "error": "failure",
                "human_control": "HumanControl",
                "rtl": "Rtl"
            },
            "Preflight": {
                "error": "failure",
            },
            "ReadMessages": {
                "error": "failure",
            },
            "ReadMessagesAirborne": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl" 
            },
            "UnstableTakeoff": {
                "error": "failure",
                "failed_takeoff": "Land",
                "human_control": "HumanControl",
            },
            "Takeoff": {
                "error": "failure",
                "failed_takeoff": "Land",
                "human_control": "HumanControl",
                "rtl": "Rtl"
            },
            "HumanControl": {
                "succeeded": "failure",
                "error": "failure",
            },
            "OnGround": {
                "error": "failure",
            },
            "PhasedCircle": {
                "error": "failure",
                "human_control": "HumanControl",
                "abort": "AbortHover",
                "rtl": "Rtl"
            },
            "PositionDrone": {
                "error": "failure",
            },
            "PossibleVictimDetected": {
                "error": "failure",
            },
            "Rtl": {
                "error": "failure",
                "human_control": "HumanControl"
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
                "human_control": "HumanControl",
                "rtl": "Rtl"
            }
        }
        print(state_factory._registry_data.dump_debug_table())
        for state in all_states:
            # get the expected transitions for this state
            tx = all_expected_transitions[state]
            # get the actual transitions for this state
            _, actual_tx = state_factory._registry_data.find(state)
            # make sure tx == actual_tx
            self.assertDictEqual(tx, actual_tx, "The default transitions for state: {} are not correct".format(state))

if __name__ == '__main__':
    unittest.main()