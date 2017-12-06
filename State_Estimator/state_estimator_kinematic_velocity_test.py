#!/usr/bin/env python
#state estimator for kinematic steering. uses imu data for longitudinal acceleration
from ukf4 import UKF
import math
import numpy as np
import scipy.linalg
import csv
import sys

def wheelangle(ster_input):
    # compute desired steering angle and steering angle rate of change

    return 0.224314009055080*ster_input-0.008867066788855

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
    # states are: x, y, heading, longitudinal velocity, steering angle, yaw rate, longitudinal acceleration
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
    print 'inputs: '+str([ster_input, thro_input])
    print 'states'+str(x_in)
    print 'dxdt: '+str(dxdt)
    # compute new state
    x_out = np.zeros(3)
    for i in range(3):
        x_out[i] = x_in[i]+dxdt[i]*timestep

    return x_out

def observe_imu(x_in,inputs):
    # states are: x, y, heading, longitudinal velocity, steering angle, yaw rate, longitudinal acceleration
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

    obs=np.zeros(4)
    obs[0]=vx
    obs[1]=-w
    obs[2]=-a*w_dot-b*w*w+(vx_dot-lr*w*w)
    obs[3]=-(b*w_dot-a*w*w+(lr*w_dot+vx*w))

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
    state_estimator.reset(X, P)

def definition():
    global lastUpdate
    global lastUpdateSet
    global ster_chl
    global thro_chl
    global last_predict_set
    global last_predict_time
    global state_estimator
    global pub
    global mutex


    # initial state and covariances
    X = np.zeros(3)
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

    r_imu = np.zeros([4, 4])

    r_imu[0][0]=0.02
    r_imu[1][0]=0.0
    r_imu[2][0]=0.0
    r_imu[3][0]=0.0

    r_imu[0][1]=r_imu[1][0]
    r_imu[1][1]=0.0018315
    r_imu[2][1]=0
    r_imu[3][1]=0.0

    r_imu[0][2]=r_imu[2][0]
    r_imu[1][2]=r_imu[2][1]
    r_imu[2][2]=0.0755
    r_imu[3][2]=-0.0821

    r_imu[0][3]=r_imu[3][0]
    r_imu[1][3]=r_imu[3][1]
    r_imu[2][3]=r_imu[3][2]
    r_imu[3][3]=0.1957

    posalert(r_imu,'initial measurement covariance')

    state_estimator = UKF(3, 4, p_err, r_imu, X, np.zeros(2),np.zeros(4),P, .001, 0.0, 2.0, iterate_new_bicycle, observe_imu)
    # global variables for steering and throttle channels
    ster_chl = 0.0
    thro_chl = 0.0
    # constants to update accel at 50hz
    lastUpdateSet = False
    lastUpdate = 0
    last_predict_set = False
    last_predict_time = 0

def forward_integrate(x_in,timestep):
    global state_estimator
    state=state_estimator.get_state()
    w_prev=state[0]
    vx_prev=state[1]
    psi_prev=x_in[2]
    lr=0.3302
    dxdt=[vx_prev*math.cos(psi_prev)-lr*w_prev*math.sin(psi_prev),
            vx_prev*math.sin(psi_prev)+lr*w_prev*math.cos(psi_prev),
            w_prev]

    x_out = np.zeros(3)
    for i in range(3):
        x_out[i] = x_in[i]+dxdt[i]*timestep

    return x_out

def main():
    #imu noise yaw rate, ax, ay
    print(sys.argv)


    tname=sys.argv[1]
    savename=sys.argv[1]+'_stateest'
    time_slam=list()
    x=list()
    y=list()
    psis=list()
    with open('Data/'+tname+'_measslam.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        # read data
        for row in reader:
            time_slam.append(float(row[0]))
            x.append(float(row[1]))
            y.append(float(row[2]))
            psis.append(float(row[3]))

    #get input data and store in list
    time_all=list()
    u0=list()
    u1=list()
    with open('Data/'+tname+'_input.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        # read data
        for row in reader:
            time_all.append(float(row[0]))
            u0.append(float(row[2]))
            u1.append(float(row[1]))

    vx=list()
    yr=list()
    ax=list()
    ay=list()
    initial_state=list()
    with open('Data/'+tname+'_traj.csv','r') as csvfile:
        reader=csv.reader(csvfile)
        row=next(reader)
        vx.append(float(row[5]))
        for i in (1,2,3):
            initial_state.append(float(row[i]))

    with open('Data/'+tname+'_measimu.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        # read data
        for row in reader:
            yr.append(float(row[1]))
            ax.append(float(row[2]))
            ay.append(float(row[3]))

    last_time=time_all[0]
    full_state=initial_state
    last_state=full_state
    outputfile=open('Data/'+savename+'.csv','wt')

    writer=csv.writer(outputfile)
    time_all=list(set(time_all))
    time_all.sort()
    for idx in range(1,len(time_all)):
        cur_time=time_all[idx]

        d_time=cur_time-last_time
        vx.append(vx[idx-1]+d_time*ax[idx-1])

        state_estimator.update_inputs([0,1],[u1[idx],u0[idx]])
        state_estimator.predict(d_time)
        state_estimator.update_measurements([0,1,2,3],[vx[idx],yr[idx],ax[idx],ay[idx]])
        state_estimator.update(np.array([1]))

        last_time=cur_time

        full_state=forward_integrate(last_state,d_time)
        full_state=full_state.tolist()
        last_state=full_state
        cur_time=[cur_time]
        cur_state=state_estimator.get_state()
        cur_state=cur_state.tolist()
        full_state=cur_time+full_state+cur_state
        covar=state_estimator.get_covar()
        posalert(covar,'Covariance matrix',cur_time)
        writer.writerow(full_state)

    outputfile.close()

if __name__ == "__main__":
    definition()
    main()
