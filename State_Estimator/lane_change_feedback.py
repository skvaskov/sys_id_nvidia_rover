#! /usr/bin/env python
import rospy
from tf import transformations as TF
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Twist, Vector3
from rover_controller.msg import MotorCommand
from rover_controller.msg import LaneChangeParameters
from math import pi, atan2, sqrt, cos,sin,atan, tan

def update_pose(data):
    #:param data: a State message describing the current state
    #:updates current_state: global variable containing [x,y,psi,w,vx]
    #:updates steering_input: recommended steering input to pololu command
    #:updates throttle_input: recommended throttle input to pololu command
    global current_pose
    current_pose[0]=data.pose.pose.position.x
    current_pose[1]=data.pose.pose.position.y
    quat = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
    euler=TF.euler_from_quaternion(quat)
    current_pose[2]=euler[2]

def update_twist(data):
    global current_twist
    current_twist[2]=data.twist.twist.angular.z
    current_twist[0]=data.twist.twist.linear.x
    current_twist[1]=data.twist.twist.linear.y

def publish_cmd_traj(Z):
    global pub_cmd
    msg=Odometry()
    msg.header.stamp=rospy.Time.now()
    odom_quat = TF.quaternion_from_euler(0., 0., Z[2])
    msg.pose.pose = Pose(Point(Z[0], Z[1], 0.), Quaternion(*odom_quat))
    msg.twist.twist = Twist(Vector3(Z[3], Z[4], 0.), Vector3(0., 0., Z[5]))
    pub_cmd.publish(msg)

def update_trajectory(data):
    global current_trajectory
    global trajectory_flag
    trajectory_flag=True
    current_trajectory=data

def follow_trajectory(data):
    global current_pose
    global current_twist
    start_time=data.header.stamp.secs+data.header.stamp.nsecs*10**-9
    t_traj=rospy.get_time()-start_time
    if t_traj>data.T:
       return [0.0395,0.0]

    Z_ff=get_Z_ff(t_traj,data)
    publish_cmd_traj(Z_ff)
    #feedback_gains
    #kpsi=12.0
    kpsi=0.0
    #key=3.0
    key=0.0
    kvy=0.0
    #kex=1.0
    kex=0.0
    err_x=cos(Z_ff[2])*(Z_ff[0]-current_pose[0])+sin(Z_ff[2])*(Z_ff[1]-current_pose[1])
    err_y=-sin(Z_ff[2])*(Z_ff[0]-current_pose[0])+cos(Z_ff[2])*(Z_ff[1]-current_pose[1])
    err_psi=minimize_angle(Z_ff[2]-current_pose[2])

    w_des=kpsi*(err_psi)+key*err_y+kvy*(Z_ff[4]-current_twist[1])+Z_ff[5]
    if Z_ff[3]==0:
        vx_des=0.0
    else:
        vx_des=kex*err_x+Z_ff[3]
        if vx_des<0.0:
            vx_des=0.0

    #update values
    throttle_input=update_throttle(current_twist,vx_des)
    steering_input=update_steering(current_twist,w_des,vx_des)
    return [steering_input,throttle_input]
def sign(x):
    if x<0.0:
         return -1.0
    else:
         return 1.0

def stop_along_trajectory(data):
    global current_pose
    global current_twist

    decel=2/1.25
    start_time=data.header.stamp.secs+data.header.stamp.nsecs*10**-9
    t_stop=rospy.get_time()-start_time-data.stop_time
    t_zero=data.V/decel
    if t_stop>t_zero:
        return [0.0395,0.0]
    else:
        t_traj=(t_stop-0.5*decel*t_stop**2/data.V)+data.stop_time

    Z_ff=get_Z_ff(t_traj,data)
    Z_ff[3]=data.V-t_stop*decel

    if Z_ff[3]==0.0:
        Z_ff[5]=0.0
    else:
        delta=atan(0.3302*Z_ff[5]/Z_ff[3])
        if abs(delta)>0.6:
             Z_ff[5]=Z_ff[3]/0.3302*tan(sign(Z_ff[5])*0.6)
    publish_cmd_traj(Z_ff)
    #feedback gains
    kpsi=5.0
    #kpsi=0.0
    key=3.0
    #key=0.0
    kvy=0.0
    kex=1.0
    #kex=0.0
    err_x=cos(Z_ff[2])*(Z_ff[0]-current_pose[0])+sin(Z_ff[2])*(Z_ff[1]-current_pose[1])
    err_y=-sin(Z_ff[2])*(Z_ff[0]-current_pose[0])+cos(Z_ff[2])*(Z_ff[1]-current_pose[1])
    err_psi=minimize_angle(Z_ff[2]-current_pose[2])


    w_des=kpsi*(err_psi)+key*err_y+kvy*(Z_ff[4]-current_twist[1])+Z_ff[5]

    if Z_ff[3]==0:
        vx_des=0.0
    else:
        vx_des=kex*err_x+Z_ff[3]
        if vx_des<0.0:
            vx_des=0.0


    #update values
    throttle_input=update_throttle(current_twist,vx_des)
    steering_input=update_steering(current_twist,w_des,vx_des)
    return [steering_input,throttle_input]

def update_throttle(current_twist,vx_des):

    #:param state: a State message describing the current state [x,y,psi,w,vx]
    #:param vx_des: a desired longitudinal velocity
    #:return: a float of the recommended throttle input to pololu command

    Kp=-0.5
    cm=(-12.5810995587748,-33.0170773577599,4.33920832891501,20.3041178298046,0.156420898500981,4.20678380627274,10.2828808092518,-0.610920415224012)

    a0=cm[0]+cm[2]*vx_des+cm[4]*vx_des*vx_des+cm[7]*current_twist[2]**2
    a1=cm[1]+cm[3]*vx_des
    a2=cm[5]+cm[6]*vx_des

    u0_guess=(-a1-sqrt(a1*a1-4*a2*a0))/(2*a2)
    if (vx_des-current_twist[0])>0.2:
        u0=-1.0
    elif (current_twist[0]-vx_des)>0.2:
        u0=3.0
    else:
        if vx_des==0.0:
            u0=0.0
        else:
            u0=u0_guess+Kp*(vx_des-current_twist[0])
    if u0>pi:
        u0=pi
    if u0<-1.0:
        u0=-1.0

    return u0

def update_steering(current_twist,w_des,vx_des):

    #:param state: a State message describing the current state [x,y,psi,w,vx]
    #:param w_des: a desired yaw rate
    #:param vx_des: a desired longitudinal velocity
    #:return: a float of the recommended steering input to pololu command

    l=0.3302

    Kp=4.0
    if current_twist[0]<0.2:
        return 0.0395

    delta_guess=atan(l*w_des/current_twist[0])


    u1_guess=(delta_guess+0.008867066788855)/0.224314009055080

    u1=u1_guess+Kp*(w_des-current_twist[2])
    if u1>3.0:
        u1=3.0
    elif u1<-3.0:
        u1=-3.0
    return u1

def minimize_angle(angle):
    while angle<-pi:
        angle=angle+2*pi
    while angle>=pi:
        angle=angle-2*pi
    return angle

def get_Z_ff(t,data):
        quat = (data.initial_pose.orientation.x,data.initial_pose.orientation.y,data.initial_pose.orientation.z,data.initial_pose.orientation.w)
    	euler=TF.euler_from_quaternion(quat)
        psi0=minimize_angle(euler[2]-data.road_heading)
        b=data.b
        Th=data.Th
        V=data.V
        lr=0.12
        Z_ff=[0.0,0.0,0.0,0.0,0.0,0.0]
        Z_ff[2]=-(Th*b+psi0)/Th**2*t**2+b*t+psi0+data.road_heading
        Z_ff[5]=-2*(Th*b+psi0)/Th**2*t+b
        Z_ff[4]=lr*Z_ff[5]
        Z_ff[3]=V
        dx=(t**8*(2*lr*Th**4*b**4 + 8*lr*Th**3*b**3*psi0 + 12*lr*Th**2*b**2*psi0**2 + 8*lr*Th*b*psi0**3 + 2*lr*psi0**4))/(48*Th**8) - (t**6*(- 9*lr*Th**6*b**4 - 12*lr*Th**5*b**3*psi0 + 9*lr*Th**4*b**2*psi0**2 + 18*lr*Th**3*b*psi0**3 + 6*lr*Th**2*psi0**4))/(36*Th**8) - (t**2*(- 3*lr*Th**8*b**2*psi0**2 + 6*lr*Th**8*b**2 + 6*V*Th**8*b*psi0 + 2*lr*Th**7*b*psi0**3 - 12*lr*Th**7*b*psi0 + 2*lr*Th**6*psi0**4 - 12*lr*Th**6*psi0**2))/(12*Th**8) + (t*(b*lr*Th**8*psi0**3 - 3*V*Th**8*psi0**2 - 6*b*lr*Th**8*psi0 + 6*V*Th**8))/(6*Th**8) + (t**4*(lr*Th**8*b**4 - 12*lr*Th**7*b**3*psi0 + 6*V*Th**7*b**2 - 6*lr*Th**6*b**2*psi0**2 - 12*lr*Th**6*b**2 + 6*V*Th**6*b*psi0 + 12*lr*Th**5*b*psi0**3 - 24*lr*Th**5*b*psi0 + 6*lr*Th**4*psi0**4 - 12*lr*Th**4*psi0**2))/(24*Th**8) - (t**5*(5*lr*Th**7*b**4 - 10*lr*Th**6*b**3*psi0 + 3*V*Th**6*b**2 - 30*lr*Th**5*b**2*psi0**2 + 6*V*Th**5*b*psi0 - 15*lr*Th**4*b*psi0**3 + 3*V*Th**4*psi0**2))/(30*Th**8) + (t**3*(3*lr*Th**8*b**3*psi0 - 3*V*Th**8*b**2 - 9*lr*Th**7*b**2*psi0**2 + 18*lr*Th**7*b**2 + 6*V*Th**7*b*psi0 - 9*lr*Th**6*b*psi0**3 + 18*lr*Th**6*b*psi0 + 6*V*Th**6*psi0**2))/(18*Th**8) - (t**7*(7*lr*Th**5*b**4 + 21*lr*Th**4*b**3*psi0 + 21*lr*Th**3*b**2*psi0**2 + 7*lr*Th**2*b*psi0**3))/(42*Th**8)
        dy= -(t**2*(6*lr*Th**6*b**2*psi0 + 3*V*Th**6*b*psi0**2 - 6*V*Th**6*b - 6*lr*Th**5*b*psi0**2 + 12*lr*Th**5*b - 6*lr*Th**4*psi0**3 + 12*lr*Th**4*psi0))/(12*Th**6) + (t**6*(- 3*V*Th**4*b**3 + 6*lr*Th**3*b**3 - 6*V*Th**3*b**2*psi0 + 18*lr*Th**2*b**2*psi0 - 3*V*Th**2*b*psi0**2 + 18*lr*Th*b*psi0**2 + 6*lr*psi0**3))/(36*Th**6) - (t*(V*Th**6*psi0**3 + 3*b*lr*Th**6*psi0**2 - 6*V*Th**6*psi0 - 6*b*lr*Th**6))/(6*Th**6) + (t**7*(V*Th**3*b**3 + 3*V*Th**2*b**2*psi0 + 3*V*Th*b*psi0**2 + V*psi0**3))/(42*Th**6) - (t**4*(V*Th**6*b**3 - 12*lr*Th**5*b**3 - 6*V*Th**5*b**2*psi0 - 6*V*Th**4*b*psi0**2 + 24*lr*Th**3*b*psi0**2 + 12*lr*Th**2*psi0**3))/(24*Th**6) - (t**5*(- 3*V*Th**5*b**3 + 15*lr*Th**4*b**3 + 30*lr*Th**3*b**2*psi0 + 6*V*Th**3*b*psi0**2 + 15*lr*Th**2*b*psi0**2 + 3*V*Th**2*psi0**3))/(30*Th**6) + (t**3*(- 3*lr*Th**6*b**3 - 3*V*Th**6*b**2*psi0 + 18*lr*Th**5*b**2*psi0 + 3*V*Th**5*b*psi0**2 - 6*V*Th**5*b + 18*lr*Th**4*b*psi0**2 + 3*V*Th**4*psi0**3 - 6*V*Th**4*psi0))/(18*Th**6)
        Z_ff[0]=data.initial_pose.position.x+cos(data.road_heading)*dx-sin(data.road_heading)*dy
        Z_ff[1]=data.initial_pose.position.y+sin(data.road_heading)*dx+cos(data.road_headin)*dy
        return Z_ff





def stop_driver(event):
    # when called, sets the commands to zero (rover no move)
    # publish steering_input
    commands=[0.0395,0.0]

    mtr=MotorCommand()
    mtr.joint_name='1'
    mtr.position=commands[0]
    mtr.speed=0.0
    mtr.acceleration=0.0
    pub.publish(mtr)
    r.sleep()
    mtr=MotorCommand()
    mtr.joint_name='0'
    mtr.position=commands[1]
    mtr.speed=0.0
    mtr.acceleration=0.0
    pub.publish(mtr)

    print "stopped"

def definition():
    #pose =vehicles x,y,heading
    #twist =vehicles vx,vy,w
    global current_trajectory
    global trajectory_flag
    global current_pose
    global current_twist
    global pub_cmd

    #set default initial values
    current_pose=[0.0,0.0,0.0]
    current_twist=[0.0,0.0,0.0]
    trajectory_flag=False

    #publisher to publish commanded trajectory
    pub_cmd=rospy.Publisher('rover/commanded_trajectory',Odometry,queue_size=10)

def main():
    global current_trajectory
    global trajectory_flag

    pub=rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
    rospy.init_node('controller')
    sub = rospy.Subscriber('rover/odometry', Odometry, update_twist)
    sub2=rospy.Subscriber('rover/odometry',Odmetry,update_pose)
    #sub2 = rospy.Subscriber('mocap', PoseStamped, update_pose)
    sub3 = rospy.Subscriber('rover/lane_change_cmd',LaneChangeParameters,update_trajectory)
    # change the duration to change how often driving commands are sent
    # no point in this being faster than the state estimator frequency
    feedback_rate=20
    r=rospy.Rate(feedback_rate)#feedback rate in hz

    print "started!"
    while not rospy.is_shutdown():
        if trajectory_flag and not current_trajectory.stop_flag:
            commands=follow_trajectory(current_trajectory)
        elif trajectory_flag and current_trajectory.stop_flag:
            commands=stop_along_trajectory(current_trajectory)
        else:
            commands=[0.0395,0.0]

        # publish steering_input
        mtr=MotorCommand()
        mtr.joint_name='1'
        mtr.position=commands[0]
        mtr.speed=0.0
        mtr.acceleration=0.0
        pub.publish(mtr)
        #print 'steering: '+str(commands[0])
        rospy.sleep(1.0/feedback_rate/2.0)
        mtr=MotorCommand()
        mtr.joint_name='0'
        mtr.position=commands[1]
        mtr.speed=0.0
        mtr.acceleration=0.0
        pub.publish(mtr)
        #print 'throttle: '+str(commands[1])
        r.sleep()

    # change the duration to change when the velocity/yaw rate commands are set to zero
    rospy.Timer(rospy.Duration(10.0), stop_driver, oneshot=True)
    rospy.spin()

if __name__ == "__main__":
    definition()
    main()
