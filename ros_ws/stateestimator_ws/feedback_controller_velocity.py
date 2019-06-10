#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from ros_pololu_servo.msg import MotorCommand
from geometry_msgs.msg import Twist, Pose, TwistStamped
from math import pi, atan2, sqrt, sin, cos


def update_throttle(state,vx_des):

    #:param state: a State message describing the current state [x,y,psi,w,vx]
    #:param vx_des: a desired longitudinal velocity
    #:return: a float of the recommended throttle input to pololu command

    Kp=-0.5
    cm=(-12.5810995587748,-33.0170773577599,4.33920832891501,20.3041178298046,0.156420898500981,4.20678380627274,10.2828808092518,-0.610920415224012)

    a0=cm[0]+cm[2]*vx_des+cm[4]*vx_des*vx_des+cm[7]*state[3]*state[3]
    a1=cm[1]+cm[3]*vx_des
    a2=cm[5]+cm[6]*vx_des

    u0_guess=(-a1-sqrt(a1*a1-4*a2*a0))/(2*a2)
    if (vx_des-state[4])>0.1:
        u0=-1.0
    elif (state[4]-vx_des)>0.3:
        u0=3.0
    else:
        if vx_des==0.0:
            u0=0.0
        else:
            u0=u0_guess+Kp*(vx_des-state[4])
    if u0>pi:
        u0=pi
    if u0<-1.0:
        u0=-1.0

    return u0

def update_steering(state,w_des,vx_des):

    #:param state: a State message describing the current state [x,y,psi,w,vx]
    #:param w_des: a desired yaw rate
    #:param vx_des: a desired longitudinal velocity
    #:return: a float of the recommended steering input to pololu command

    l=0.3302

    Kp=0.25

    delta_guess=atan2(l*w_des,vx_des)

    u1_guess=(delta_guess+0.008867066788855)/0.224314009055080

    u1=u1_guess+Kp*(w_des-state[3])
    if u1>3.0:
        u1=3.0
    elif u1<-3.0:
        u1=-3.0

    return u1

def drive(event):
    #regularly scheduled event to publish control inputs
    #if the feedback topic has not been after a while publish 0's
    global steering_input
    global throttle_input
    global feedback_catch_time
    global feedback_rate
    global last_state
    global reset_twist
    global reset_pose
    global pub 

    c_time=rospy.Time.now()-feedback_catch_time
    if (c_time.to_sec())<0.5:
        # publish steering_input
        mtr=MotorCommand()
        mtr.joint_name='1'
        mtr.position=steering_input
        mtr.speed=0.0
        mtr.acceleration=0.0
        pub.publish(mtr)
        print 'steering: '+str(steering_input)
    	rospy.sleep(1.0/feedback_rate/2.0)
        # publish throttle_input
        mtr=MotorCommand()
        mtr.joint_name='0'
        mtr.position=throttle_input
        mtr.speed=0.0
        mtr.acceleration=0.0
        pub.publish(mtr)
        print 'throttle: '+str(throttle_input)
    else:
        print 'timed out'
        # publish 0 steering_input
        mtr=MotorCommand()
        mtr.joint_name='1'
        mtr.position=0.0395
        mtr.speed=0.0
        mtr.acceleration=0.0
        pub.publish(mtr)
        print 'steering: '+str(steering_input)
        rospy.sleep(1.0/feedback_rate/2.0)
        # publish 0 throttle_input
        mtr=MotorCommand()
        mtr.joint_name='0'
        mtr.position=0.0
        mtr.speed=0.0
        mtr.acceleration=0.0
        pub.publish(mtr)
        print 'throttle: '+str(throttle_input)
        reset_msg=Twist()
        reset_msg.linear.x=0.0
        reset_msg.angular.z=0.0
        reset_twist.publish(reset_msg)
        reset_msg=Pose()
        reset_msg.position.x=last_state[0]
        reset_msg.position.y=last_state[1]
        reset_msg.orientation.w=cos(last_state[2]/2)
        reset_msg.orientation.x=sin(last_state[2]/2)
        reset_pose.publish(reset_msg)

def update_state(data):
    #:param data: a State message describing the current state
    #:updates current_state: global variable containing [x,y,psi,w,vx]
    #:updates steering_input: recommended steering input to pololu command
    #:updates throttle_input: recommended throttle input to pololu command

    global current_state
    global commanded_velocity
    global commanded_yaw_rate
    global steering_input
    global throttle_input
    global state_initialized
    global last_state

    if state_initialized==False:
         current_state=[0,0,0,0,0]
         last_state=current_state
         state_initialized=True

    current_state[0]=data.pose.pose.position.x
    current_state[1]=data.pose.pose.position.y
    qw=data.pose.pose.orientation.w
    qx=data.pose.pose.orientation.x
    qy=data.pose.pose.orientation.y
    qz=data.pose.pose.orientation.z
    current_state[2]=atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    current_state[3]=data.twist.twist.angular.z
    current_state[4]=data.twist.twist.linear.x

    steering_input=update_steering(current_state,commanded_yaw_rate,commanded_velocity)
    throttle_input=update_throttle(current_state,commanded_velocity)

def update_commands(data):
    #:param data: a State message describing the current state
    #:updates commanded_velocity, commanded_yaw_rate: global variables containing commands
    #:updates steering_input: recommended steering input to pololu command
    #:updates throttle_input: recommended throttle input to pololu command
    #:updates last_state: last state while recieving commands
    global feedback_catch_time
    global commanded_velocity
    global commanded_yaw_rate
    global steering_input
    global throttle_input
    global current_state
    global last_state
    
    last_state=current_state
    feedback_catch_time=rospy.Time.now()
    commanded_velocity=data.twist.linear.x
    commanded_yaw_rate=data.twist.angular.z

    steering_input=update_steering(current_state,commanded_yaw_rate,commanded_velocity)
    throttle_input=update_throttle(current_state,commanded_velocity)

def definition():

    #current_state: [x,y,psi,w,vx]
    #steering_input: recommended steering input to pololu command
    #throttle_input: recommended throttle input to pololu command
    #commanded_velocity: desired velocity
    #commanded_yaw_rate: desired yaw rate
    #feedback_catch_time: keeps track of when we are recieving commands
    #feedback_rate: rate in hz we want the loop to run in
    #pub: publisher for motor controller

    global current_state
    global commanded_velocity
    global commanded_yaw_rate
    global steering_input
    global throttle_input
    global pub
    global reset_pose
    global reset_twist
    global feedback_catch_time
    global feedback_rate
    global state_initialized
    #set default initial values
    state_initialized=False
    steering_input=0.0395
    throttle_input=0.0
    commanded_velocity=0.0
    commanded_yaw_rate=0.0
    feedback_catch_time=0.0
    feedback_rate=10
    pub=rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
    reset_pose=rospy.Publisher('/rover/reset_pose',Pose, queue_size=10)
    reset_twist=rospy.Publisher('/rover/reset_twist',Twist, queue_size=10)

def main():
    global feedback_catch_time
    global feedback_rate

    rospy.init_node('controller')
    feedback_catch_time=rospy.Time.now()

    sub = rospy.Subscriber('rover/odometry', Odometry, update_state)
    sub2 = rospy.Subscriber('rover/cmd_vel', TwistStamped, update_commands)

    # change the duration to change how often driving commands are sent
    # no point in this being faster than the state estimator frequency
    driver = rospy.Timer(rospy.Duration(1.0/feedback_rate), drive)
    print "started!"

    rospy.spin()

if __name__ == "__main__":
    definition()
    main()
