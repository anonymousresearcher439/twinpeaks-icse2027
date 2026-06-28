from abc import ABC, abstractmethod
import enum
from queue import Queue
from threading import Event, Lock, Thread
from typing import Any, Callable, Mapping, NewType, Optional, Tuple, Union

from .frames import AttitudeData
from .kinematics import EnuYaw


# TODO: create centralized location for dr-onboard application specific types
# quaternions are represented x, y, z, w (to match what the tf library does)
Quaternion = NewType("Quaternion", Tuple[float, float, float, float])

# 0 index is a unique message identifier and 1 index is the message data
Message = NewType("Message", Tuple[Union[str, int], Any])


class GimbalStatus(enum.Enum):
    CONNECTED_BUT_NOT_IN_CONTROL = enum.auto()
    CONNECTED_WITH_ERROR = enum.auto()
    NOT_CONNECTED = enum.auto()
    READY = enum.auto()
    UNKNOWN = enum.auto()


class GimbalAxes(enum.Flag):
    PITCH = enum.auto()
    ROLL = enum.auto()
    YAW = enum.auto()


# a gimbal object controls only a single gimbal
# a gimbal object only stores details relevant to communicating
# with a single gimbal
class Gimbal(ABC):
    """An abstract class defining the minimum expected Gimbal interfaces
    
    A Gimbal object only controls a single gimbal device

    It is expected that the instance variable _current_attitude will be populated continually
    with the latest known value

    Commands will be added to a queue and sent in their own thread

    Gimbal related messages will be processed and added to their own queue via a separate
    thread. A third thread will read from the message queue and apply callback functions based on 
    the unique tag given to each message type as they are placed in the queue
    """
    def __init__(self):
        """__init__ will at least:
        - initialize the base class
        - set _gimbal_axes
        - aquire control of or initialize communication with a gimbal
        """
        self.attitude = AttitudeData()
        self.heading: Optional[EnuYaw] = None

        self._current_attitude: Optional[Quaternion] = None
        self._current_attitude_lock: Lock = Lock()
        self._gimbal_axes: Optional[GimbalAxes] = None

        self._process_command_queue_thread = Thread(
            target=self._manage_thread_shutdown,
            kwargs={"function_to_loop": self._process_command_queue},
            daemon=True
        )
        self._process_messages_thread = Thread(
            target=self._manage_thread_shutdown,
            kwargs={"function_to_loop": self._process_messages},
            daemon=True
        )

        self._command_queue: "Queue[Any]" = Queue()
        self._received_messages_queue: "Queue[Message]" = Queue()

        self._process_command_queue_thread.start()
        self._process_messages_thread.start()

        self._exit_event = Event()


    def _manage_thread_shutdown(self, function_to_loop: Callable, kwargs: Optional[Mapping]=None):
        """Loops indefinitely on function_to_loop until exit event triggers break
        """
        while True:
            if kwargs is not None:
                function_to_loop(**kwargs)
            else:
                function_to_loop()

            if self._exit_event.is_set():
                break


    def _on_exit(self):
        """This method will be called when stop_communication is called
        """
        pass


    @abstractmethod
    def _process_command_queue(self):
        """Extracts commands from the command queue and sends them. This function is already wrapped
        in while True
        """
        pass


    @abstractmethod
    def _process_messages(self):
        """Extracts Messages from the _received_messages_queue and runs callback functions mapped to 
        the message identifiers against the associated message data. This function is already wrapped
        in while True
        """
        pass


    @abstractmethod
    def receive_message_callback(self, message: Any):
        """Receives gimbal related message and places it in the _received_messages_queue.
        """
        pass


    @abstractmethod
    def get_gimbal_status(self) -> GimbalStatus:
        """Check whether the gimbal is ready to receive commands
        """
        pass


    def get_attitude(self) -> Optional[Quaternion]:
        """Returns the most recent known gimbal attitude relative to its local coordinate frame
        """
        with self._current_attitude_lock:
            return self._current_attitude


    def get_gimbal_axes(self) -> GimbalAxes:
        return self._gimbal_axes


    @abstractmethod
    def reaquire_control(self):
        """Reaquires the connection and / or control of the initialized gimbal
        """
        pass


    @abstractmethod
    def set_attitude(self, attitude: Quaternion):
        """Set the attitude of the gimbal. 
        
        The quaternion parameter is specified using the convention from TF (X, Y, Z, W).
        The quaternion applies rotation in a forward (x, roll) right (y, roll) down (z, yaw)
        coordinate system.
        """
        pass


    @abstractmethod
    def set_neutral_attitude(self):
        """Sets the gimbal to its defined neutral attitude
        """
        pass


    def stop_communication(self):
        """Shut down all gimbal communication and message processing threads. Call this message when
        the Gimbal is no longer needed. 
        """
        self._on_exit()
        self._exit_event.set()


