
import numpy as np
import pandas as pd
from dataLoader import dataLoader

ld = dataLoader()

def solvePointCloudReg_Arun(a, b):
    '''Method due to K.Arun, et. al., IEEE PAMI, Vol 9, no 5, pp 698-700, Sept 1987
        @param a: input 3nx3 np.mat b = Fa
        @param b: input 3nx3 np.mat b = Fa
        @return: R, p components of F
    '''
    a_mean = a.mean(0)
    # print(a_mean)
    b_mean = b.mean(0)

    a_hat = np.mat(a - a_mean)
    b_hat = np.mat(b - b_mean)
    # print(a_hat[1].T * b_hat[1])
    # compute H
    H = []
    for i in range(len(a)):
        H.append(a_hat[i].T * b_hat[i])
    H = np.vstack(sum(H))
    # print(H)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T * U.T
    # print(U,'\n', Vt,'\n', H)
    det = np.linalg.det(R)
    # print(R)
    if det < 0:
        V = Vt.T * [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
        R = V * U.T
    p = np.mat(b_mean).T - R * np.mat(a_mean).T
    F = [R, p]
    return F

def pivotCalibration(frames):
    '''
    @param num_frame: number of frames
    @param markers: numpy array of markers coordinates [m1; m2; m3; m4; m5]
    '''
    R = []
    p = []
    I = []
    num_frame = len(frames)
    # get mean point of initial frame markers
    G_0 = frames[0].mean(0)
    for j in range(len(frames[0])):
        gi = frames[0] - G_0
    for i in range(num_frame):
        Gi = frames[i]
        F = solvePointCloudReg_Arun(gi, Gi)
        R.append(F[0])
        p.append(F[1])
        I.append(np.identity(3))
    R = np.vstack(R)
    p = np.vstack(p)
    I = np.vstack(I)
    # print('init frame',gi)
    #set a lstsq problem
    X = np.hstack((I, -R))
    # print(R)
    w = np.linalg.lstsq(X, p, None)[0]
    p_dimple = w[:3]
    t_tip = w[3:]
    t_relative = t_tip.T - G_0
    # print('t_tip', t_tip)
    print(G_0, t_tip)
    return p_dimple, t_relative

if __name__ == '__main__':
    csv_data = pd.read_csv('drill_calibration/drill_calibration_1.csv')
    fs = ld.frames(int(len(csv_data)/6), csv_data)
    print(int(len(csv_data)/10))
    # print('fs',fs[0])
    p, t = pivotCalibration(fs)
    print('p',p, '\nt', t)
# pivot_calibration_2022.2.2/pivot_calibration_with_geometry_1.csv