from dataclasses import dataclass
from typing import Optional, Union, TypeVar, Type

import rospy
from redis import Redis

from cyphal_bridge.msg import HMILed, HMIBeeper
from command_processor.srv import DriveState, DriveStateRequest, DriveStateResponse


@dataclass
class Gear:
    max_linear: float  # m/s
    max_angular: float  # rad/s


T = TypeVar('T')

class State:
    GEARS = {
        1: Gear(0.2, 0.4),
        2: Gear(0.4, 0.8),
        3: Gear(1.0, 2.0)
    }

    def __init__(self, autoload=True):
        self.rds: Redis = Redis(host='localhost', port=6379, decode_responses=True)

        if autoload:
            self._gear: int = self.load_int("gear", 1, save_on_miss=True)
            self._mode: int = self.load_int("mode", 1, save_on_miss=True)
        else:
            self._gear: int = 1
            self._mode: int = 1

        self.hmi_led = rospy.Publisher('/hmi/led', HMILed, queue_size=10)
        self.hmi_beeper = rospy.Publisher('/hmi/beeper', HMIBeeper, queue_size=10)

        self.send_mode_indicators(self._mode)

    @property
    def max_linear_speed(self):
        return self.GEARS[self.gear].max_linear

    @property
    def max_angular_speed(self):
        return self.GEARS[self.gear].max_angular

    @property
    def gear(self) -> int:
        return self._gear

    @gear.setter
    def gear(self, value: int):
        self._gear = value
        self.store("gear", value)

    @property
    def mode(self) -> int:
        return self._mode

    @mode.setter
    def mode(self, value: int):
        if self._mode != value:
            self.send_mode_indicators(value)
        self._mode = value
        self.store("mode", value)

    def send_mode_indicators(self, for_mode):
        if for_mode == 1:
            self.hmi_led.publish(HMILed(r=0, g=0, b=255))
            self.hmi_beeper.publish(HMIBeeper(duration=-1, frequency=0.5))
        else:
            self.hmi_led.publish(HMILed(r=0, g=255, b=0))
            self.hmi_beeper.publish(HMIBeeper(duration=1, frequency=10))

    def redis_get(self, name, convert_to: Type[T] = str, default=None, save_on_miss=False) -> Optional[T]:
        val = self.rds.get(name)
        if val is None:
            if default is not None:
                if save_on_miss:
                    self.store(name, default)
                return default
            return None
        try:
            return convert_to(val)
        except (TypeError, ValueError) as e:
            raise TypeError(f"Could not convert <{val}> to <{convert_to}> due to <{e}>")

    def load_int(self, name, default, save_on_miss=False) -> Optional[int]:
        return self.redis_get(name, int, default, save_on_miss=save_on_miss)

    def load_float(self, name, default, save_on_miss=False) -> Optional[float]:
        return self.redis_get(name, float, default, save_on_miss=save_on_miss)

    def store(self, name, value: Union[str, int, float]):
        self.rds.set(name, str(value))


state: Optional[State] = None

def get_state():
    return state


def handle_drive_state(req: DriveStateRequest):
    if state is None:
        return DriveStateResponse(ok=False)

    if req.gear not in (0, 1, 2, 3) or req.mode  not in (0, 1, 2):
        return DriveStateResponse(ok=False)
    if req.gear != 0:
        state.gear = req.gear
    if req.mode != 0:
        state.mode = req.mode

    return DriveStateResponse(ok=True)


def setup_state():
    global state
    state = State()
    drive_state_service = rospy.Service('drive_state', DriveState, handle_drive_state)
    return drive_state_service
