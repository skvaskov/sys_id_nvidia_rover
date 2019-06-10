#!/usr/bin/env python
#state estimator for kinematic steering. uses imu data for longitudinal acceleration
from ukf4 import UKF
import rospy
import math
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Header
from threading import Lock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from ros_pololu_servo.msg import MotorCommand

def gen_state():
    global state_estimator
    global current_pose
    x = state_estimator.get_state()
    p = state_estimator.get_covar()
    posalert(p,'covariance')
    msg = Odometry()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x=current_pose[0]
    msg.pose.pose.position.y=current_pose[1]
    msg.pose.pose.position.z=0.0
    msg.pose.pose.orientation.w=math.cos(current_pose[2]/2)
    msg.pose.pose.orientation.x=0.0
    msg.pose.pose.orientation.y=0.0
    msg.pose.pose.orientation.z=math.sin(current_pose[2]/2)
    msg.pose.covariance=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0,   0.01, 0.0, 0.0, 0.0,
                               0.0,0.0,    0.0,   0.005,0.0,0.0,
                               0.0,0.0,  0.0,    0.0, 0.005,0.0,
                               0.0,0.0,  0.0,    0.0, 0.0, 0.005)
    msg.twist.twist.linear.x=x[1]
    msg.twist.twist.linear.y=0.12*x[0]
    msg.twist.twist.angular.z=x[0]
    msg.twist.covariance=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0,   0.01, 0.0, 0.0, 0.0,
                               0.0,0.0,    0.0,   0.005,0.0,0.0,
                               0.0,0.0,  0.0,    0.0, 0.005,0.0,
                               0.0,0.0,  0.0,    0.0, 0.0, 0.005)
    odom_broadcast = tf2_ros.TransformBroadcaster()
    trans = geometry_msgs.msg.TransformStamped()
    trans.header.stamp = msg.header.stamp
    trans.header.frame_id = msg.header.frame_id
    trans.child_frame_id = msg.child_frame_id
    trans.transform.translation = msg.pose.pose.position
    trans.transform.rotation = msg.pose.pose.orientation
    odom_broadcast.sendTransform(trans)
    return msg

def wheelangle(ster_input):
    # compute desired steering
    delta= 0.224314009055080*ster_input-0.008867066788855
    return delta 

def drivingforce(thro_input,vx,w):
    #driving force and computation
    if thro_input <= -0.35:
        cm = np.zeros(8)
        cm[0]=-12.5810995587748
        cm[1]=-33.0170773577599
        cm[2]=4.33920832891501
        cm[3]=20.3041178298046
        cm[4]=0.156420898500981
        cm[5]=4.20678380627274
        cm[6]=10.2828808092518
        cm[7]=-0.610920415224012
        Frx = cm.dot(np.array([1, thro_input, vx, vx*thro_input, vx*vx, thro_input*thro_input, vx*thro_input*thro_input, w*w]))

    elif thro_input > 0.0:
        cm = np.zeros(3)
        cm[0] =  -4.11177295309464
        cm[1] = -15.1817204116634
        cm[2] =  5.22364002070909
        Frx = cm.dot(np.array([1, vx, vx * vx]))
    else:
        if vx > 0.05:
            cm = np.zeros(4)
            cm[0] = -5.55660998280113
            cm[1] = -13.8953541919073
            cm[2] =  -2.47286920126272
            cm[3] = 0.480990612787014
            Frx = cm.dot(np.array([1, thro_input, vx, vx * vx]))
        elif vx < -0.05:
            Frx = 0.5
        else:
            Frx=0.0

    return Frx

def iterate_new_bicycle(x_in, timestep, inputs):
    # states are:yaw rate, long velocity, wheel angle
    ster_input=inputs[0]
    thro_input=inputs[1]

    w_prev = x_in[0]
    vx_prev = x_in[1]
    delta_prev = x_in[2]

    # vehicle parameters
    m = 7.78
    Iz = 0.2120
    l = 0.3302
    lr = 0.12
    mo = (m*lr*lr+Iz)/(l*l)

    delta_des=wheelangle(ster_input)

    k_st = 4.300730919846748
    delta_dot = k_st * (delta_des - delta_prev)

    Frx=drivingforce(thro_input, vx_prev,w_prev)
    vx_dot=(Frx-mo*math.tan(delta_prev)/(math.cos(delta_prev)*math.cos(delta_prev))*delta_dot*vx_prev)/(m+mo*math.tan(delta_prev)*math.tan   (delta_prev))

    # compute rate of change of first five states
    dxdt = [vx_dot/l*math.tan(delta_prev)+vx_prev/l*delta_dot/(math.cos(delta_prev)*math.cos(delta_prev)),
	        vx_dot,
            delta_dot]
    #print 'inputs: '+str(inputs)
    #print 'wheelcmd: '+str(delta_des)
    #print 'state: '+str(x_in)
    #print 'dxdt: '+str(dxdt)
    # compute new state
    x_out = np.zeros(3)
    for i in range(3):
        x_out[i] = x_in[i]+dxdt[i]*timestep

    return x_out

def observe_imu(x_in,inputs):
    
    ster_input=inputs[0]
    thro_input=inputs[1]

    vx = x_in[1]
    w =x_in[0]
    delta = x_in[2]

    # vehicle parameters
    m = 7.78
    Iz = 0.2120
    l = 0.3302
    lr = 0.12
    mo = (m*lr*lr+Iz)/(l*l)
    a= 0.048
    b= -0.0543

    delta_des=wheelangle(ster_input)

    k_st = 4.300730919846748
    delta_dot = k_st * (delta_des - delta)

    Frx=drivingforce(thro_input, vx,w)
    vx_dot=(Frx-mo*math.tan(delta)/(math.cos(delta)*math.cos(delta))*delta_dot*vx)/(m+mo*math.tan(delta)*math.tan(delta))
    w_dot=vx_dot/l*math.tan(delta)+vx/l*delta_dot/(math.cos(delta)*math.cos(delta))

    obs=np.zeros(3)
    obs[0]=-w
    obs[1]=-a*w_dot-b*w*w+(vx_dot-lr*w*w)
    obs[2]=-(b*w_dot-a*w*w+(lr*w_dot+vx*w))

    return obs

def forward_integrate(pose_in,timestep):
    global state_estimator
    state=state_estimator.get_state()
    w_prev=state[0]
    vx_prev=state[1]
    psi_prev=pose_in[2]
    lr=0.3302
    dxdt=[vx_prev*math.cos(psi_prev)-lr*w_prev*math.sin(psi_prev),
            vx_prev*math.sin(psi_prev)+lr*w_prev*math.cos(psi_prev),
            w_prev]

    pose_out = np.zeros(3)
    for i in range(3):
        pose_out[i] = pose_in[i]+dxdt[i]*timestep

    return pose_out

def posalert(Q,name,t=-1):
    egs=np.linalg.eigvals(Q)
    neg= False
    zr=False
    for eg in egs:
        if eg<0:
            neg= True
        if eg==0:
            zr=True
    if neg:
        if t>0:
            print(name+' has a negative eigien value at time '+str(t))
        else:
            print(name+' has a negative eigen value')
    if zr:
        if t>0:
            print(name+' has a zero eigien value at time '+str(t))
        else:
            print(name+' has a zero eigen value')


def quat2Rot(q):
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]

    R = np.zeros([3, 3])
    R[0][0] = 1-2 * (y * y + z * z)
    R[1][0] = 2 * (x * y + z * w)
    R[2][0] = 2 * (x * z - y * w)

    R[0][1] = 2 * (x * y - z * w)
    R[1][1] = 1-2 * (x * x + z * z)
    R[2][1] = 2 * (y * z + x * w)

    R[0][2] = 2 * (x * z + y * w)
    R[1][2] = 2 * (y * z - x * w)
    R[2][2] = 1-2 * (x * x + y * y)
    return R

def reset_pose(data):
    global current_pose
    current_pose[0]=data.position.x
    current_pose[1]=data.position.y
    qw=data.orientation.w
    qx=data.orientation.x
    qy=data.orientation.y
    qz=data.orientation.z
    current_pose[2]=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    print 'Pose reset'

def reset_twist(data):
    global state_estimator
    X=np.zeros(3)
    X[0]=data.angular.z
    X[1]=data.linear.x
    inputs=state_estimator.get_inputs()
    X[2]=wheelangle(inputs[0])

    P = np.eye(3)
    P[0][0] = 0.05
    P[1][1] = 0.05
    P[2][2] = 0.05
    P *= 0.0001
    state_estimator.reset(X, P,[0.0,0.0,0.0])
    print 'state estimator reset'

def update_inputs(data):
   global state_estimator
   if data.joint_name=='1':
        state_estimator.update_inputs([0],[data.position])
   if data.joint_name=='0': 
        state_estimator.update_inputs([1],[data.position])

def general_update(event):
    global state_estimator
    global current_pose
    global mutex
    global last_predict_time
    global last_predict_set
    global imu_catch_time
    global pub_odometry

    mutex.acquire()
    if not last_predict_set:
        last_predict_time= rospy.Time.now()
        last_predict_set= True

    
    d_time = rospy.Time.now() - last_predict_time
    current_pose=forward_integrate(current_pose,d_time.to_sec())
    state_estimator.predict(d_time.to_sec())
    last_predict_time =last_predict_time+d_time
    m_time=rospy.Time.now() - imu_catch_time
    if (m_time.to_sec())<0.2:
        state_estimator.update(np.array([0,1,2]))
    pub_odometry.publish(gen_state())
    mutex.release()
    print 'Position: '+str(current_pose)
    print 'States: '+str(state_estimator.get_state())
  

def update_raw_accel(data):  
    global imu_catch_time
    global state_estimator
    m_time=rospy.Time.now() - imu_catch_time
    if (m_time.to_sec())<0.02:
        return

    imu_catch_time=rospy.Time.now()
    qw=data.orientation.w
    qx=data.orientation.x
    qy=data.orientation.y
    qz=data.orientation.z
    R=quat2Rot([qw,qx,qy,qz])
    O=np.transpose(R)
    araw=np.array([data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z])
    g=np.array([0.0,0.0,9.8065])
    acc=araw-O.dot(g)
    state_estimator.update_measurements([0,1,2],[data.angular_velocity.z, acc[0],acc[1]])
    

def definition():
    global last_predict_set
    global last_predict_time
    global current_pose
    global imu_catch_time
    global state_estimator
    global pub_odometry
    global pub_odom_trans
    global mutex
    
    # initial state and covariances
    X = np.zeros(3)
    current_pose=[0.0,0.0,0.0]
    P = np.eye(3)
    P[0][0] = 0.05
    P[1][1] = 0.05
    P[2][2] = 0.05
    P *= 0.0001

    p_err = np.zeros([3, 3])

    # process noise for bicycle model (assumed 5deg noise for steering angle)

    p_err[0][0] = 0.041427307254232
    p_err[1][0] = 0.000433545671265
    p_err[2][0] = 0.009394468994580

    p_err[0][1] = p_err[1][0]
    p_err[1][1] = 0.012935785843696
    p_err[2][1] = 0.000226132365965

    p_err[0][2] = p_err[2][0]
    p_err[1][2] = p_err[2][1]
    p_err[2][2] = 0.016585577808243


    posalert(p_err,'inital process covariance')
    #noise for measurements 
    r_imu = np.zeros([3, 3])

    r_imu[0][0]=0.0018315
    r_imu[1][0]=0.0
    r_imu[2][0]=0.0

    r_imu[0][1]=r_imu[1][0]
    r_imu[1][1]=0.5755
    r_imu[2][1]=-0.0821

    r_imu[0][2]=r_imu[2][0]
    r_imu[1][2]=r_imu[2][1]
    r_imu[2][2]=0.6957

    posalert(r_imu,'initial measurement covariance')
    # global variables for predicition updates
    last_predict_set = False
    last_predict_time=0.0
    imu_catch_time=0.0
    pub_odometry = rospy.Publisher('/rover/odometry', Odometry, queue_size=10)
    state_estimator = UKF(3, 3, p_err, r_imu, X, [0.0,0.0],[0.0,0.0,0.0],P, .001, 0.0, 2.0, iterate_new_bicycle, observe_imu)
    mutex = Lock()

def main():
    global last_predict_time
    global imu_catch_time

    estimator_rate=20
    rospy.init_node('state_estimator')
    last_predict_time = rospy.Time.now()
    imu_catch_time=rospy.Time.now() 
    np.set_printoptions(precision=3, suppress=True)
    seq = 0
    sub = rospy.Subscriber('/imu/imu', Imu, update_raw_accel)
    sub1 = rospy.Subscriber('/pololu/command', MotorCommand, update_inputs)
    sub2 = rospy.Subscriber('/rover/reset_twist',Twist, reset_twist)
    sub3 = rospy.Subscriber('/rover/reset_pose',Pose,reset_pose)
    estimator=rospy.Timer(rospy.Duration(1.0/estimator_rate), general_update)
    rospy.spin()


if __name__ == "__main__":
    definition()
    main()
