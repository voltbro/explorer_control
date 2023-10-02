import time

import rospy
from std_msgs.msg import Float64

from .state_control import setup_state
from .wheels_control import setup_wheels, get_speeds, get_last_msg_time


is_zeroed = False
ZEROING_NS = 1000000000 * 0.3


def main():
    global is_zeroed
    rospy.init_node("command_processor")
    drive_state_srv = setup_state()
    cmd_sub, left_wheels, right_wheels, all_wheels = setup_wheels()

    print("Starting")
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        if (time.monotonic_ns() - get_last_msg_time()) < ZEROING_NS:
            left_speed, right_speed = get_speeds()
            left_wheels.publish(left_speed)
            right_wheels.publish(right_speed)
            is_zeroed = False
        elif not is_zeroed:
            all_wheels.publish(Float64(0))
            is_zeroed = True
        rate.sleep()
