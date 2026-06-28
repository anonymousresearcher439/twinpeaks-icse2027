"""State Factory

This module provides a decorator that can be used to register a class under
one or more names. Once registered, the class can be accessed by name using
the `get_state` function.

If the same name is registered more than once, a warning is printed at the time
of registration. If you try to access a name that has been registered more than
once, an exception is raised.

It's important that we do not raise an exception at the time of registration so
that we can identify all name conflicts before the program starts.

Furthermore we have a special function that reports all name conflicts
and prints the file and class name for each conflicting registration.
"""
import copy
import inspect
import os

from typing import (
    Dict,
    List,
    Optional,
    Tuple,
    Type,
    TypedDict,
    TypeVar,
)

import rospy
import smach


STANDARD_TRANSITIONS_FLYING = {
    "error": "failure",
    "failsafe": "Failsafe",
    "abort": "AbortHover",
    "rtl": "Rtl",
    "low_battery": "ReturnToRecharge"
}

BASIC_TRANSITIONS = {"error": "failure"}


class _RegistryData:
    """
    A private class used to maintain a mapping from state names to state classes.

    At the heart of this class is a dictionary that maps state names to class objects.

    As part of the mission parsing process, we need a way to initialize the states that are specified in the mission. The mission will name the type of state, and we use this name to identify a class object. Elsewhere we then initialize the state by calling the class constructor. But the _RegistryData class is focused on mapping names to classes.

    This class is used internally within this module. It gives us a way to
    register a state's class object with one or more names. When we register a state,
    we're just adding an entry to the dictionary. we register each state with one or more
    unique names so that a mission can use it.

    The main job of this class is to build and maintain this mapping from names
    to classes. This mapping is used in our factory function to create instances
    of the appropriate state classes when needed.

    In addition to this main job, this class also provides robust error handling.
    It checks for these potential problems:

    1. We look up a name that has not been registered. In this case, an error is
    raised and we print lots of details. 

    2. We register the same name more than once. In this case, we output a
    detailed warning message and the program continues.

    3. We register the same name more than once, and then we try to access that
    name. In this case, an exception is raised and we output lots of details.

    Detailed error messages are an essential part of this class. When something
    goes wrong, we can output an ASCII table that lists:

    - The name that was registered
    - The class that was registered
    - The file where the is was defined
    """

    class DuplicateRegistration:
        """A class used to represent a duplicate registration."""
        def __bool__(self):
            return False

    DUPLICATE = DuplicateRegistration()

    class NameNotFoundError(KeyError):
        """A class used to represent a name that was not registered."""
        pass
    
    class DataStructureError(Exception):
        """A class used to represent a data structure error."""
        pass

    def __init__(self):
        self._state_name_map = {}
        self._default_transitions = {}
        self._conflict_records = {}
    
    def register(self, name: str, cls: Type, default_transitions: Dict[str, str], overwrite: bool = False):
        """Add a name to class mapping to the registry.
        """
        if name in self._state_name_map and not overwrite:
            self._register_name_conflicts(name, cls)
            
        else:
            self._state_name_map[name] = cls
            self._default_transitions[name] = default_transitions

    def _register_name_conflicts(self, name, cls):
        """Register a name conflict and output a warning.

        This method is called when we try to register a name that has already
        been registered. We record the details of this and we output a warning
        message.
        """
        # First we record the details of this conflict.
        # We've either seen this name before, or not.
        if name in self._conflict_records:
            # If we've seen it before, then we can just append the new class to
            # the existing list of classes.
            self._conflict_records[name].append(cls)
        else:
            # If we haven't seen this name before, then we need to create a list
            # and add both classes to it.
            first_cls = self._state_name_map[name]
            self._conflict_records[name] = [first_cls, cls]
            self._state_name_map[name] = _RegistryData.DUPLICATE
            self._default_transitions[name] = _RegistryData.DUPLICATE

        # Next we output a detailed warning message.
        conflict_details = []
        for cls in self._conflict_records[name]:
            record = [name, cls.__name__, inspect.getfile(cls)]
            conflict_details.append(record)
        # add headings to the first row
        conflict_details.insert(0, ["Registration Name", "Class", "File"])
        table = _build_ascii_table(conflict_details)
        rospy.logwarn(f"ERROR: name conflicts found. The name: '{name}' was registered more than once!{os.linesep + os.linesep}{table}" )
        
    
    def find(self, name: str) -> Tuple[Type, Dict[str, str]]:
        """Find the class object and default transitions for the given name.

        If the name has not been registered, then an exception is raised. If the
        name has been registered more than once, then an exception is raised.
        """
        if name not in self._state_name_map:
            # in this case we need to output all known names and classes

            all_records = []
            for key, cls in self._state_name_map.items():
                all_records.append([key, cls.__name__, inspect.getfile(cls)])
            all_records.insert(0, ["Registration Name", "Class", "File"])
            table = _build_ascii_table(all_records)
            line_indent = os.linesep + '    '
            rospy.logerr(f"ERROR: State name not found: '{name}'.{line_indent}We could not find a state with the name: '{name}'.{line_indent}For reference here is every name we know about:{os.linesep + os.linesep}{table}" )
            raise _RegistryData.NameNotFoundError(f"ERROR: State name not registered: '{name}'")
        if self._state_name_map[name] is _RegistryData.DUPLICATE:
            # in this case we need to output all known classes for the given name
            conflict_records = []
            for cls in self._conflict_records[name]:
                conflict_records.append([name, cls.__name__, inspect.getfile(cls)])
            conflict_records.insert(0, ["Registration Name", "Class", "File"])
            table = _build_ascii_table(conflict_records)
            line_indent = os.linesep + '    '
            rospy.logerr(f"ERROR: State name registered more than once: '{name}'.{line_indent}We could not find a unique state named '{name}' because multiple classes were registered with that name.{line_indent}Here is every class that was registered with this name:{os.linesep + os.linesep}{table}" )
            raise ValueError(f"ERROR: State name registered more than once: '{name}'")
        
        cls_obj = self._state_name_map[name]
        transitions = copy.copy(self._default_transitions[name])

        return cls_obj, transitions
    
    def dump_debug_table(self)-> str:
        """create a table with every name and class that's been registered."""
        
        # we need to include all names, even the ones that have been registered
        # more than once. So we need to merge the records from _state_name_map with
        # the records from _conflict_records
        table_data = []

        # first lets add all the records from _state_name_map
        for key, cls in self._state_name_map.items():
            if cls: # filter DUPLICATE entries
                table_data.append([key, cls.__name__, inspect.getfile(cls)])
        
        # next we add all the records from _conflict_records (these are the DUPLICATE entries)
        for key, cls_list in self._conflict_records.items():
            for cls in cls_list:
                table_data.append([key, cls.__name__, inspect.getfile(cls)])
        
        # next we need to sort the records by name, then by class name
        table_data.sort(key=lambda x: (x[0], x[1]))

        # add headings to the first row
        table_data.insert(0, ["Registration Name", "Class", "File"])

        # next we create the table
        table = _build_ascii_table(table_data)
        return table

    # this method checks for name conflicts and raises an exception if it finds any
    # it will output the dump_debug_table if it finds any conflicts
    def check_for_name_conflicts(self):
        """Check for name conflicts and raise an exception if any are found.

        This method is called after all states have been registered. It checks
        for name conflicts and raises an exception if any are found. It also
        outputs a table with all the names and classes that have been registered.
        """
        # make sure the _conflict_records is empty
        if self._conflict_records:
            duplicates = self._conflict_records.keys()
            duplicate_list = "\n - ".join(duplicates)
            table = self.dump_debug_table()
            error_message = f"""ERROR: the same name was registered more than once!
            
The following names were registered more than once:

 - {duplicate_list}

Here is a table with every registered name and class:

{table}
            """
            rospy.logerr(error_message)
            raise _RegistryData.DataStructureError("ERROR: the same name was registered more than once!")


_registry_data = _RegistryData()


def overwrite_state_registration(name: str, cls: Type, default_transitions: Dict[str, str]):
    """Overwrite an existing state registration with a new state class.

    Args:
        name (str): The name of the state to overwrite.

        cls (Type): The class to associate with this name. This class must be a subclass of BaseState or it must implement a compatible interface.

        default_transitions (Dict[str, str]): A dictionary mapping outcomes to state names.

    This function allows you to replace a state registration with a new one. It is particularly useful when we need a good default state registration for the Takeoff class but we need a custom Takeoff state for ardupilot copters.
    """
    rospy.loginfo(f"Overwriting state registration for {name}")
    _registry_data.register(name, cls, default_transitions, overwrite=True)


def register_state(*state_names: str, default_transitions: Dict[str, str] = None):
    """Register a state with its default transitions.

    The state_names are strings. They are the names that a mission can use to
    identify your state. You can register a state with zero or more names. A
    mission would then use one of these names to identify your state. If you
    don't specify any names, then the class name is used as the name of the
    state. For example, if the class name is Arm, then the state is registered 
    under the name "Arm" if no state names are specified. If you register a
    state with more than one name then a mission can use any of those names to
    identify the this Class. 

    The default_transitions argument is a dictionary that maps transition names
    to state names. The transition name is a string and it's the outcome that is
    returned by this state's execute method. The state name is the name of
    another state that's a part of the mission. If the state returns the given
    outcome, then the mission will transition to the state with the given name.
    The state name must identify a state that's part of the mission. By default,
    these states are provided in all missions:

    - "AbortHover"
    - "Failsafe"
    - "Rtl"

    In case the state machine should terminate for a given outcome  then the
    the outcome should map to one of the following special names:
    - "failure"
    - "mission_completed"
    """
    def decorator(cls: Type):
        global _registry_data
        nonlocal state_names
        # by default use the class name as the state name
        if not state_names:
            state_names = [cls.__name__]

        for name in state_names:
            _registry_data.register(name, cls, default_transitions)
        
        return cls
        
    return decorator


class StateSpec(TypedDict):
    """
    A dictionary that defines everything needed to call smach.StateMachine.add()

    This dictionary is used when building a state machine.
    """
    label: str
    state: smach.State
    transitions: Dict[str, str]


def init_state(state_name: str, transition_spec: List, args: Dict, class_name: Optional[str] = None) -> StateSpec:
    if class_name is None:
        class_name = state_name
    Constructor, transitions = _registry_data.find(class_name)
    for transition in transition_spec:
        transitions[transition["condition"]] = transition["target"]
    
    all_outcomes = set()
    for outcome in transitions:
        all_outcomes.add(outcome)
    
    if "outcomes" in args:
        for outcome in args['outcomes']:
            all_outcomes.add(outcome)
    args['outcomes'] = list(all_outcomes)

    rospy.loginfo(f"Found Constructor for {state_name}: {Constructor}")
    rospy.loginfo(f"Building {class_name} with transitions: {transitions}")
    return {
        'label': state_name,
        'state': Constructor(**args),
        'transitions': transitions
    }


def _build_ascii_table(table_data: List[List[str]]) -> str:
    """Build an ascii table from a list of rows.

    A row is a list of strings. Each string is a column in the table.
    """
    def _find_col_widths(all_rows) -> List[int]:
        """Find the column widths we need to build an ASCII table.
        """
        result = []
        for row in all_rows:
            for i, field in enumerate(row):
                if len(result) <= i:
                    result.append(len(field))
                previous_biggest = result[i]
                result[i] = max(previous_biggest, len(field))

        return result
    
    table = []
    table.append(table_data[0]) # add headings

    # next we add a row of dashes to separate the headings from the data
    dashes = []
    col_widths = _find_col_widths(table_data)

    for field_width in col_widths:
        dashes.append("-"*field_width)
    table.append(dashes)

    table.extend(table_data[1:])

    table_str = ""
    for row in table:
        middle = " | ".join((val.ljust(width) for val, width in zip(row, col_widths)))
        table_str += "| " + middle + " |\n"
    return table_str
