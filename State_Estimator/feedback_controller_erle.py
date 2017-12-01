#!/usr/bin/env python

import rospy
from erle_python_testing.msg import State
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float64
from math import pi, atan


def control_velocity(state):
    """
    Uses the globally known vel_command and outputs a recommended throttle to achieve that velocity
    :param state: a State message describing the current state
    :return: a float of the recommended throttle
    """
    Kp = 100
    Kd = 0
    throttle_guess = (vel_command + 44.709) / 0.028307

    if (vel_command-state.v)>.2:
        ret=throttle_guess+60
    elif (state.v-vel_command)> .2:
        ret=throttle_guess-250
    else:
        ret=throttle_guess+(vel_command - state.v) * Kp - state.v_dot * Kd

    ret=ret if ret < 1700 else 1700
    ret=ret if ret > 1300 else 1300
    return ret


def control_yawrate(state, yrr):
    """
    Uses the globally known yr_command and outputs a recommended steering input to achieve that velocity
    :param state: a State message describing the current state
    :param yrr: the current rate of change of the yaw rate
    :return: a float of the recommended steering rc input
    """
    Kp = 100
    Kd = 0
    delta = atan(0.29 * yr_command / vel_command) if vel_command != 0.0 else 0.0
    if delta > 0:
        steer_guess = (delta - 1.8202133) / -0.0011682083
    elif delta < 0:
        steer_guess = (delta - 1.8237556) / -0.0011384355
    else:
        steer_guess = 1590

    dev = (state.psi_dot - yr_command) * Kp + yrr * Kd
    ret = steer_guess + dev
    ret = ret if ret < 1950 else 1950
    ret = ret if ret > 1050 else 1050
    return ret


def update_control(data):
    # callback for new state information

    global throttle
    global steer
    global prev_state

    throttle = control_velocity(data)
    steer = control_yawrate(data, (data.psi_dot - prev_state.psi_dot) / 0.02)
    prev_state = data


def drive(event):
    # regularly scheduled callback to output the current recommended rc inputs

    global msg

    # this must be done every time
    change_mode(custom_mode="MANUAL")

    msg.channels[2] = int(throttle)
    msg.channels[0] = int(steer)

    # publish control inputs
    print "commanded:", msg.channels[2], msg.channels[0]
    pub.publish(msg)


def update_vel_cmd(data):
    # callback for a new velocity command
    global vel_command
    vel_command = data.data


def update_yr_cmd(data):
    # callback for a new yaw rate command
    global yr_command
    yr_command = data.data


def stop_driver(event):
    # when called, sets the commands to zero (rover no move)
    global vel_command
    global yr_command

    vel_command = 0.0
    yr_command = 0.0

    print "stopped"

rospy.init_node('controller')
sub = rospy.Subscriber('state', State, update_control)

prev_state = State()
prev_state.psi_dot = 0

# if no commands are coming in, this will be used as the default command
# can use this to provide static commands for a fixed period of time
vel_command = 0.0
yr_command = 0.0

default_throttle = 1500
default_steer = 1590
throttle = 1500
steer = 1590

change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
sub2 = rospy.Subscriber('vel_cmd', Float64, update_vel_cmd)
sub3 = rospy.Subscriber('yr_cmd', Float64, update_yr_cmd)
msg = OverrideRCIn()

# create initial control input
msg.channels = [1500] * 8
msg.channels[2] = 1590

# change the duration to change how often driving commands are sent
# no point in this being faster than the state estimator frequency
driver = rospy.Timer(rospy.Duration(0.02), drive)
print "started!"

# change the duration to change when the velocity/yaw rate commands are set to zero
rospy.Timer(rospy.Duration(3.0), stop_driver, oneshot=True)

rospy.spin()
