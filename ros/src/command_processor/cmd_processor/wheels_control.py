import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from .state_control import get_state


WHEEL_RADIUS = 0.18
R = 0.305


def ms_to_rad(ms):
    return ms / WHEEL_RADIUS


def wheel_speed(v_a, v_l, is_inner):
    return R * abs(v_a) * (-1 if is_inner else 1) + v_l


last_msg_time, linear_speed, angular_speed = 0, 0, 0


def get_last_msg_time():
    return last_msg_time


def get_speeds():
    state = get_state()
    if state is None or state.mode == 1:
        return 0, 0
    lin_s = linear_speed * state.max_linear_speed
    ang_s = angular_speed * state.max_angular_speed
    left_ms = wheel_speed(ang_s, lin_s, ang_s < 0)
    right_ms = wheel_speed(ang_s, lin_s, ang_s > 0)
    return Float64(ms_to_rad(left_ms)), Float64(ms_to_rad(right_ms))


def cmd_callback(cmd: Twist):
    global linear_speed, angular_speed, last_msg_time
    last_msg_time = time.monotonic_ns()
    linear_speed = cmd.linear.x
    angular_speed = cmd.angular.z


class PublisherCollection:
    def __init__(self, *pubs):
        self.pubs = pubs

    def publish(self, *args, **kwargs):
        for pub in self.pubs:
            pub.publish(*args, **kwargs)


def setup_wheels():
    cmd_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_callback)

    pub_br = rospy.Publisher('/wheel_vel/br', Float64, queue_size=10)
    pub_bl = rospy.Publisher('/wheel_vel/bl', Float64, queue_size=10)
    pub_fr = rospy.Publisher('/wheel_vel/fr', Float64, queue_size=10)
    pub_fl = rospy.Publisher('/wheel_vel/fl', Float64, queue_size=10)

    left_wheels = PublisherCollection(pub_bl, pub_fl)
    right_wheels = PublisherCollection(pub_br, pub_fr)
    all_wheels = PublisherCollection(pub_br, pub_fr, pub_fl, pub_bl)

    return cmd_sub, left_wheels, right_wheels, all_wheels
