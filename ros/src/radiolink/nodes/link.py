#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from cyphal_bridge.srv import ResetDrive
from command_processor.srv import DriveState
from cyphal_bridge.msg import HMIBeeper, HMILed

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node("radiolink")

cmd = Twist()

last_gear_signal = None
last_reset_signal = None
last_mode_signal = None

reset_drives = rospy.ServiceProxy('reset_drives', ResetDrive)
drive_state = rospy.ServiceProxy('drive_state', DriveState)


def send_reset():
    try:
        reset_drives(0)
    except rospy.ServiceException as e:
        print("Reset drives call failed: %s" % e)


def parse_signal(signal, intervals=(1, 2, 3)):
    if signal is None:
        return 0
    res_sig = 1
    if -0.01 < signal < 0.01:
        res_sig = intervals[1]
    elif signal < 0:
        res_sig = intervals[0]
    else:
        res_sig = intervals[2]
    return res_sig


def send_state(gear_signal, mode_signal):
    gear = parse_signal(gear_signal)
    mode = parse_signal(mode_signal, (1, 1, 2))

    try:
        drive_state(gear=gear, mode=mode)
    except rospy.ServiceException as e:
        print("Drive state call failed: %s" % e)


def joy_callback(data: Joy):
    global last_gear_signal, last_reset_signal, last_mode_signal
    cmd.linear.x = -data.axes[2]
    cmd.angular.z = -data.axes[0]

    should_send_state = False

    gear_signal = data.axes[6]
    mode_signal = data.axes[4]
    reset_signal = data.axes[5]

    if gear_signal != last_gear_signal:
        last_gear_signal = gear_signal
        should_send_state = True
    else:
        gear_signal = None

    if mode_signal != last_mode_signal:
        last_mode_signal = mode_signal
        should_send_state = True
    else:
        mode_signal = None

    if should_send_state:
        send_state(gear_signal, mode_signal)

    if reset_signal != last_reset_signal:
        last_reset_signal = reset_signal
        if reset_signal < 0:
            send_reset()


rospy.wait_for_service('/reset_drives')
rospy.wait_for_service('/drive_state')

rospy.Subscriber("joy", Joy, joy_callback)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    pub.publish(cmd)
    rate.sleep()
