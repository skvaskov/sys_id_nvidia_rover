#!/usr/bin/env python
#state estimator for kinematic steering. uses imu data for longitudinal acceleration
from ukf2 import UKF
import math
import numpy as np
import scipy.linalg
import csv

def wheelangle(ster_input):
    # compute desired steering angle and steering angle rate of change

    return 0.224314009055080*ster_input-0.008867066788855

def drivingforce(thro_input,vx):
    #driving force and computation
    if thro_input <= -0.35:
        cm = np.zeros(7)
        cm[0] =  -32.3444674451266
        cm[1] = -107.4936376966535
        cm[2] =   5.4559263010323
        cm[3] = 35.5572000100370
        cm[4] =   0.7595495751754
        cm[5] = -48.4489780846895
        cm[6] = 21.5881438769874

        Frx = cm.dot(np.array([1, thro_input, vx, vx*thro_input, vx*vx, thro_input*thro_input, vx*thro_input*thro_input]))
    elif thro_input >= 1.0:
        cm = np.zeros(3)
        cm[0] =  -5.345435991947356
        cm[1] = -11.906846985500659
        cm[2] =  3.564802758464205
        Frx = cm.dot(np.array([1, vx, vx * vx]))
    else:
        if vx > 0.05:
            cm = np.zeros(3)
            cm[0] = -5.767811170782461
            cm[1] =  0.378942271548532
            cm[2] = -0.346210024796956
            Frx = cm.dot(np.array([1, vx, vx * vx]))
        elif vx < -0.05:
            cm = np.zeros(3)
            cm[0] = 5.767811170782461
            cm[1] = -0.378942271548532
            cm[2] = 0.346210024796956
            Frx = cm.dot(np.array([1, vx, vx * vx]))
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

    Frx=drivingforce(thro_input, vx_prev)
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

    Frx=drivingforce(thro_input, vx)
    vx_dot=(Frx-mo*math.tan(delta)/(math.cos(delta)*math.cos(delta))*delta_dot*vx)/(m+mo*math.tan(delta)*math.tan(delta))
    w_dot=vx_dot/l*math.tan(delta)+vx/l*delta_dot/(math.cos(delta)*math.cos(delta))

    obs=np.zeros(3)
    obs[0]=-w
    obs[1]=b*w_dot+a*w*w-(vx_dot-lr*w*w)
    obs[2]=-a*w_dot+b*w*w-(lr*w_dot+vx*w)

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
    p_err[0][0] = 0.00079272
    p_err[1][0] = -0.00053722
    p_err[2][0] = 0.000096637
    p_err[3][0] = 0.0036741
    p_err[4][0] = 0.00037926
    p_err[5][0] = 0.00018292

    p_err[0][1] = p_err[1][0]
    p_err[1][1] = 0.00081119
    p_err[2][1] = -0.000024471
    p_err[3][1] = -0.0038486
    p_err[4][1] = 0.00025713
    p_err[5][1] = -0.00024970

    p_err[0][2] = p_err[2][0]
    p_err[1][2] = p_err[2][1]
    p_err[2][2] = 0.00028007
    p_err[3][2] = -0.000025277
    p_err[4][2] = -0.000048306
    p_err[5][2] = 0.000032992

    p_err[0][3] = p_err[3][0]
    p_err[1][3] = p_err[3][1]
    p_err[2][3] = p_err[3][2]
    p_err[3][3] = 0.041699
    p_err[4][3] = 0.00075822
    p_err[5][3] = 0.0068806

    p_err[0][4] = p_err[4][0]
    p_err[1][4] = p_err[4][1]
    p_err[2][4] = p_err[4][2]
    p_err[3][4] = p_err[4][3]
    p_err[4][4] = 0.012855
    p_err[5][4] = -0.00030286

    p_err[0][5] = p_err[5][0]
    p_err[1][5] = p_err[5][1]
    p_err[2][5] = p_err[5][2]
    p_err[3][5] = p_err[5][3]
    p_err[4][5] = p_err[5][4]
    p_err[5][5] = 0.0026761

    #imu noise yaw rate, ax, ay

    r_imu = np.zeros([3, 3])

    r_imu[0][0]=0.0018315
    r_imu[1][0]=0.0040356
    r_imu[2][0]=-0.0037105

    r_imu[0][1]=r_imu[1][0]
    r_imu[1][1]=0.26871
    r_imu[2][1]=-0.026971

    r_imu[0][2]=r_imu[2][1]
    r_imu[1][2]=r_imu[2][2]
    r_imu[2][2]=0.11932


    posalert(R,'initial measurement covariance')
    posalert(p_err,'inital process covariance')

    state_estimator = UKF(6, 3, p_err, X, P, .001, 0.0, 2.0, iterate_new_bicycle, observe_imu)
    # global variables for steering and throttle channels
    ster_chl = 0.0
    thro_chl = 0.0
    # constants to update accel at 50hz
    lastUpdateSet = False
    lastUpdate = 0
    last_predict_set = False
    last_predict_time = 0

def main():

    tname='real_sturn'
    savename='real_sturn_stateest'
    time_slam=list()
    x=list()
    y=list()
    psis=list()
    with open('data/'+tname+'_measslam.csv', 'r') as csvfile:
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
    with open('data/'+tname+'_input.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        # read data
        for row in reader:
            time_all.append(float(row[0]))
            u0.append(float(row[1]))
            u1.append(float(row[2]))

    vx=list()
    yr=list()
    ax=list()
    ay=list()
    with open('data/'+tname+'_measimu.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        # read data
        for row in reader:
            yr.append(float(row[1]))
            ax.append(float(row[2]))
            ay.append(float(row[3]))

    last_time=time_all[0]
    outputfile=open('data/'+savename+'.csv','wt')

    writer=csv.writer(outputfile)
    time_all=list(set(time_all))
    time_all.sort()

    for idx in range(1,len(time_all)):
        cur_time=time_all[idx]
        #reset state to slam data
        # for si in range(len(time_slam)):
        #     if time_slam[si]==cur_time:
        #         cur_state=state_estimator.get_state()
        #         rs=np.array([x[si],y[si],psis[si],cur_state[3],cur_state[4],cur_state[5],cur_state[6]])
        #         state_estimator.reset(rs,p_err)
        d_time=cur_time-last_time
        inputs=np.array([u1[idx],u0[idx]])
        state_estimator.predict(d_time,inputs)
        imu_data=np.array([yr[idx],ax[idx],ay[idx]])
        state_estimator.update([0,1,2],imu_data,R,inputs)
        last_time=cur_time

        cur_state=state_estimator.get_state()
        cur_state=cur_state.tolist()
        cur_state.insert(0,cur_time)
        covar=state_estimator.get_covar()
        posalert(covar,'Covariance matrix',cur_time)
        writer.writerow(cur_state)

    outputfile.close()

if __name__ == "__main__":
    definition()
    main()
