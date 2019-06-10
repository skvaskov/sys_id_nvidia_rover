#!/usr/bin/env python
#state estimator for kinematic steering. uses imu data for longitudinal acceleration
from ukf3 import UKF
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Header
from threading import Lock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from ros_pololu_servo.msg import MotorCommand

def gen_state():
    global state_estimator
    x = state_estimator.get_state()
    p = state_estimator.get_covar()
    posalert(p,'covaiance')
    msg = Odometry()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x=x[0]
    msg.pose.pose.position.y=x[1]
    msg.pose.pose.position.z=0.0
    msg.pose.pose.orientation.w=math.cos(x[2]/2)
    msg.pose.pose.orientation.x=0.0
    msg.pose.pose.orientation.y=0.0
    msg.pose.pose.orientation.z=math.sin(x[2]/2)
    msg.pose.covariance=(p[0][0], p[0][1], 0.0, 0.0, 0.0, p[0][2], 
                         p[1][0], p[1][1], 0.0, 0.0, 0.0, p[1][2], 
                         0.0,     0.0,     0.0, 0.0, 0.0, 0.0,  
                         0.0,     0.0,     0.0, 0.0, 0.0, 0.0,
                         0.0,     0.0,     0.0, 0.0, 0.0, 0.0,
                         p[2][0], p[2][1], 0.0, 0.0, 0.0, p[2][2])
    msg.twist.twist.linear.x=x[4]
    msg.twist.twist.linear.y=0.12*x[3]
    msg.twist.twist.linear.z=0.0
    msg.twist.twist.angular.x=0.0
    msg.twist.twist.angular.y=0.0
    msg.twist.twist.angular.z=x[3]
    msg.twist.covariance=(p[4][4],       0.0144*p[4][3], 0.0, 0.0, 0.0, p[4][3],
                          0.0144*p[4][3],0.0144*p[3][3], 0.0, 0.0, 0.0, 0.0114*p[3][3],
                          0.0,           0.0,            0.0, 0.0, 0.0, 0.0,
                          0.0,           0.0,            0.0, 0.0, 0.0, 0.0,
                          0.0,           0.0,            0.0, 0.0, 0.0, 0.0,
                          p[4][3],       0.0114*p[3][3], 0.0, 0.0, 0.0, p[3][3])
    return msg

def wheelangle(ster_input):
    # compute desired steering
    delta=0.0
    if (ster_input<0.0) & (ster_input>0.05):
        delta= 0.224314009055080*ster_input-0.008867066788855
    return delta 

def drivingforce(thro_input,vx,w):
    #driving force and computation
    if thro_input <= -0.35:
        cm = np.zeros(8)
        #cm[0] = -25.1220434879362
        #cm[1] = -81.7158714056317
        #cm[2] =  2.08170809764619
        #cm[3] = 21.1235710569551
        #cm[4] = 0.237666708745836
        #cm[5] = -31.5571659476673
        #cm[6] = 12.6153618800570
        #cm[7] = -0.610920415224012
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
    # states are: x, y, heading, longitudinal velocity, steering angle, yaw rate, longitudinal acceleration
    ster_input=inputs[0]
    thro_input=inputs[1]

    psi_prev = x_in[2]
    w_prev = x_in[3]
    vx_prev = x_in[4]
    delta_prev = x_in[5]

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
    dxdt = [vx_prev*math.cos(psi_prev)-lr*w_prev*math.sin(psi_prev),
            vx_prev*math.sin(psi_prev)+lr*w_prev*math.cos(psi_prev),
            w_prev,
            vx_dot/l*math.tan(delta_prev)+vx_prev/l*delta_dot/(math.cos(delta_prev)*math.cos(delta_prev)),
	    vx_dot,
            delta_dot]

    # compute new state
    x_out = np.zeros(6)
    for i in range(6):
        x_out[i] = x_in[i]+dxdt[i]*timestep

    return x_out

def observe_imu(x_in,inputs):
    # states are: x, y, heading, longitudinal velocity, steering angle, yaw rate, longitudinal acceleration
    ster_input=inputs[0]
    thro_input=inputs[1]

    psi = x_in[2]
    vx = x_in[4]
    w =x_in[3]
    delta = x_in[5]

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
    obs[1]=b*w_dot+a*w*w+(vx_dot-lr*w*w)
    obs[2]=a*w_dot-b*w*w-(lr*w_dot+vx*w)

    return obs

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


def reset_state(data):
    global state_estimator
    X=np.zeros(6)
    X[0]=data.position.x
    X[1]=data.position.y
    qw=data.orientation.w
    qx=data.orientation.x
    qy=data.orientation.y
    qz=data.orientation.z
    X[2]=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
    X[3]=0.0
    X[4]=0.0
    inputs=state_estimator.get_inputs()
    X[5]=wheelangle(inputs[0])

    P = np.eye(6)
    P[0][0] = 1
    P[1][1] = 1
    P[2][2] = 0.05
    P[3][3] = 0.05
    P[4][4] = 0.05
    P[5][5] = 0.05
    P *= 0.0001
    state_estimator.reset(X, P)
    print 'state estimator reset'

def update_inputs(data):
   global state_estimator
   if data.joint_name=='1':
        state_estimator.update_inputs([0],[data.position])
   if data.joint_name=='0': 
        state_estimator.update_inputs([1],[data.position])
   general_update()
	
def general_update():
    global state_estimator
    global mutex
    global last_predict_time
    global last_predict_set
    global imu_catch_time
    global pub
    #imu noise yaw rate, ax, ay
    r_imu=np.zeros([3,3])
    r_imu[0][0]=0.0018315
    r_imu[1][0]=0
    r_imu[2][0]=-0.0037105

    r_imu[0][1]=r_imu[1][0]
    r_imu[1][1]=0.26871
    r_imu[2][1]=1
    #r_imu[2][1]=-0.026971

    r_imu[0][2]=r_imu[2][1]
    r_imu[1][2]=r_imu[2][2]
    #r_imu[2][2]=0.11932
    r_imu[2][2]=1
    mutex.acquire()
    if not last_predict_set:
        last_predict_time= rospy.Time.now()
        last_predict_set= True

    
    d_time = rospy.Time.now() - last_predict_time
    state_estimator.predict(d_time.to_sec())
    last_predict_time =last_predict_time+d_time
    m_time=rospy.Time.now() - imu_catch_time
    if (m_time.to_sec())<0.2:
        #state_estimator.update([0,1,2],r_imu)
        state_estimator.update([0],[r_imu[0][0]])
    pub.publish(gen_state())
    mutex.release()
    print state_estimator.get_state()
  

def update_raw_accel(data):  
    global imu_catch_time
    global state_estimator
    imu_catch_time=rospy.Time.now()
    state_estimator.update_measurements([0,1,2],[data.angular_velocity.z, data.linear_acceleration.x,data.linear_acceleration.y])
   
def definition():
    global last_predict_set
    global last_predict_time
    global imu_catch_time
    global state_estimator
    global pub
    global mutex


    # initial state and covariances
    X = np.zeros(6)
    P = np.eye(6)
    P[0][0] = 1
    P[1][1] = 1
    P[2][2] = 0.05
    P[3][3] = 0.05
    P[4][4] = 0.05
    P[5][5] = 0.05
    P *= 0.0001

    p_err = np.zeros([6, 6])

    # process noise for bicycle model (assumed 5deg noise for steering angle)
    p_err[0][0] = 0.000277960733394
    p_err[1][0] = 0.000006956239161
    p_err[2][0] = -0.000076867462869
    p_err[3][0] = 0.000183068978775
    p_err[4][0] = -0.000000135904073
    p_err[5][0] =  0.000029998898969

    p_err[0][1] = p_err[1][0]
    p_err[1][1] =  0.000210616111621
    p_err[2][1] = 0.000119817678146
    p_err[3][1] = -0.000066884429792
    p_err[4][1] =  0.000030257928822
    p_err[5][1] = -0.000003714940906

    p_err[0][2] = p_err[2][0]
    p_err[1][2] = p_err[2][1]
    p_err[2][2] = 0.000650910699953
    p_err[3][2] = -0.000261134322014
    p_err[4][2] = -0.000247267037420
    p_err[5][2] = -0.000024430905112

    p_err[0][3] = p_err[3][0]
    p_err[1][3] = p_err[3][1]
    p_err[2][3] = p_err[3][2]
    p_err[3][3] = 0.041427307254232
    p_err[4][3] =  0.000433545671265
    p_err[5][3] =  0.009394468994580

    p_err[0][4] = p_err[4][0]
    p_err[1][4] = p_err[4][1]
    p_err[2][4] = p_err[4][2]
    p_err[3][4] = p_err[4][3]
    p_err[4][4] = 0.012935785843696
    p_err[5][4] = 0.000226132365965

    p_err[0][5] = p_err[5][0]
    p_err[1][5] = p_err[5][1]
    p_err[2][5] = p_err[5][2]
    p_err[3][5] = p_err[5][3]
    p_err[4][5] = p_err[5][4]
    p_err[5][5] = 0.016585577808243

    posalert(p_err,'inital process covariance')
   
    # global variables for predicition updates
    last_predict_set = False
    last_predict_time=0.0
    imu_catch_time=0.0
    pub = rospy.Publisher('/rover/state', Odometry, queue_size=10)
    state_estimator = UKF(6, 3, p_err, X, [0.0,0.0],[0.0,0.0,0.0],P, .001, 0.0, 2.0, iterate_new_bicycle, observe_imu)
    mutex = Lock()

def main():
    global last_predict_time
    global imu_catch_time

    rospy.init_node('state_estimator')
    last_predict_time = rospy.Time.now()
    imu_catch_time=rospy.Time.now() 
    np.set_printoptions(precision=3, suppress=True)
    seq = 0
    sub = rospy.Subscriber('/imu/imu', Imu, update_raw_accel)
    sub1 = rospy.Subscriber('/pololu/command', MotorCommand, update_inputs)
    sub2 = rospy.Subscriber('/rover/reset_state',Pose, reset_state)
    rospy.spin()


if __name__ == "__main__":
    definition()
    main()
