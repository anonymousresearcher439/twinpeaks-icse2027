import unittest
from unittest.mock import patch
from dr_onboard_autonomy import state_factory


class TestStateFactory(unittest.TestCase):
    def setUp(self):
        # Setup code goes here
        pass

    def tearDown(self):
        # Teardown code goes here
        pass

    def test_init_RegistryData(self):
        # Test the __init__ function of the _RegistryData class
        data = state_factory._RegistryData()
        self.assertIsInstance(data, state_factory._RegistryData)
        self.assertIsInstance(data._state_name_map, dict)
        self.assertIsInstance(data._default_transitions, dict)
        self.assertIsInstance(data._conflict_records, dict)
    
    @patch.object(state_factory, 'rospy')
    def test_RegistryData_register_same_state_twice(self, mock_rospy):

        # Test the __init__ function of the _RegistryData class
        data = state_factory._RegistryData()
        class TestState1:
            pass
        data.register("Test", TestState1, {})

        class TestState2:
            pass

        data.register("Test", TestState2, {})
        call_args = mock_rospy.logwarn.call_args
        assert "ERROR: name conflicts found. The name: 'Test' was registered more than once!" in call_args[0][0]

    def test_RegistryData_register_one_state(self):
        # Test the __init__ function of the _RegistryData class
        data = state_factory._RegistryData()
        class TestState1:
            pass
        data.register("Test", TestState1, {})

        self.assertEqual(len(data._state_name_map), 1)
        self.assertEqual(len(data._default_transitions), 1)
        self.assertEqual(len(data._conflict_records), 0)

        pass

    def test_RegistryData_register_two_states_and_access_one(self):

        data = state_factory._RegistryData()
        class TestState1:
            pass
        test1_transitions = {"a": "b"}
        data.register("TEST_1_EXAMPLE", TestState1, test1_transitions)

        class TestState2:
            pass
        data.register("Test2", TestState2, {})
        # Test the _StateRegistry class
        actual_cls, actual_transitions = data.find("TEST_1_EXAMPLE")

        self.assertEqual(actual_cls, TestState1)
        self.assertEqual(actual_transitions, test1_transitions)

    @patch.object(state_factory, 'rospy')
    def test_RegistryData_register_two_states_with_same_name_and_access_the_name(self, mock_rospy):

        data = state_factory._RegistryData()
        class TestState1:
            pass
        data.register("test", TestState1, {})

        class TestState2:
            pass
        data.register("test", TestState2, {})
        # Test the _StateRegistry class
        with self.assertRaises(ValueError):
            result = data.find("test")

        call_args = mock_rospy.logerr.call_args
        actual_error_message = call_args[0][0]
        assert "ERROR: State name registered more than once:" in actual_error_message
        # make sure the classes are listed
        assert "TestState1" in actual_error_message
        assert "TestState2" in actual_error_message
        # make sure module file is listed
        assert __file__ in actual_error_message
    
    @patch.object(state_factory, 'rospy')
    def test_RegistryData_register_states_and_access_a_missing_name(self, mock_rospy):

        data = state_factory._RegistryData()
        class TestState1:
            pass
        data.register("test1", TestState1, {})

        class TestState2:
            pass
        data.register("test2", TestState2, {})
        # Test the _StateRegistry class
        with self.assertRaises(state_factory._RegistryData.NameNotFoundError):
            result = data.find("missing_name_test")

        call_args = mock_rospy.logerr.call_args
        actual_error_message = call_args[0][0]
        assert "name not found:" in actual_error_message
        # make sure we list every name that was registered
        assert "test1" in actual_error_message
        assert "test2" in actual_error_message
        # make sure every class that was registered is listed
        assert "TestState1" in actual_error_message
        assert "TestState2" in actual_error_message
        # make sure module files are listed
        assert __file__ in actual_error_message
    
    @patch.object(state_factory, 'rospy')
    def test_RegistryData_register_many_states_and_many_duplicates_and_get_debug_table(self, mock_rospy):
        class TestState1:
            pass
        class TestState2:
            pass
        class TestState3:
            pass
        class TestDuplicateA1:
            pass
        class TestDuplicateA2:
            pass
        class TestDuplicateB1:
            pass
        class TestDuplicateB2:
            pass
        class TestDuplicateB3:
            pass

        data = state_factory._RegistryData()
        data.register("test1", TestState1, {})
        data.register("test2", TestState2, {})
        data.register("test3", TestState3, {})
        data.register("test_duplicate_a", TestDuplicateA1, {})
        data.register("test_duplicate_a", TestDuplicateA2, {})
        data.register("test_duplicate_b", TestDuplicateB1, {})
        data.register("test_duplicate_b", TestDuplicateB2, {})
        data.register("test_duplicate_b", TestDuplicateB3, {})

        table = data.dump_debug_table()
        # make sure every name is listed in the table
        names = ["test1", "test2", "test3", "test_duplicate_a", "test_duplicate_b"]
        for name in names:
            assert name in table
        
        # make sure every class is listed in the table
        classes = ["TestState1", "TestState2", "TestState3", "TestDuplicateA1", "TestDuplicateA2", "TestDuplicateB1", "TestDuplicateB2", "TestDuplicateB3"]
        for cls in classes:
            assert cls in table
        
        # make this module file is listed in the table
        assert __file__ in table

        with self.assertRaises(state_factory._RegistryData.DataStructureError):
            data.check_for_name_conflicts()

        actual_error_message = mock_rospy.logerr.call_args[0][0]
        # make sure all names and classes are in the error message
        for name in names:
            assert name in actual_error_message
        for cls in classes:
            assert cls in actual_error_message

    def test_register_state_decorator(self):
        data = state_factory._RegistryData()
        # need to patch the _registry_data variable in the state_factory module
        # before we use the decorator

        # I want to patch state_factory._registry_data with the local variable data
        with patch.object(state_factory, '_registry_data', new=data):
            transitions = {
                "a": "1",
                "b": "2",
            }
            @state_factory.register_state("test", "test2", default_transitions=transitions)
            class TestState:
                def __init__(self):
                    self.x = 1
                pass
            
            @state_factory.register_state(default_transitions={"x": "y"})
            class DefaultNameTestState:
                pass

        # make sure we added records for test and test2
        self.assertEqual(len(data._state_name_map), 3)
        self.assertEqual(len(data._default_transitions), 3)
        self.assertEqual(len(data._conflict_records), 0)

        # find DefaultNameTestState, make sure we end up with the right class object, and the right transitions. Then initalize DefaultNameTestState and make sure it works
        actual_default_name_test_state_type, actual_transitions = data.find("DefaultNameTestState")
        self.assertEqual(actual_default_name_test_state_type, DefaultNameTestState)
        self.assertDictEqual(actual_transitions, {"x": "y"})
        test_instance = actual_default_name_test_state_type()
        self.assertIsInstance(test_instance, DefaultNameTestState)

        actual_test_type, actual_transitions = data.find("test")
        self.assertEqual(actual_test_type, TestState)
        my_instance = actual_test_type()
        self.assertIsInstance(my_instance, TestState)
        self.assertEqual(actual_transitions, transitions)
        actual_test2_type, actual_transitions = data.find("test2")
        self.assertEqual(actual_test2_type, TestState)
        self.assertEqual(actual_transitions, transitions)

        data.check_for_name_conflicts()
        table = data.dump_debug_table()
    



# This allows the test to be run directly
if __name__ == '__main__':
    unittest.main()