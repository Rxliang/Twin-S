from scipy.spatial.transform import Rotation as R
import numpy as np
import math
import json

def makeSkew(a):

    sk = np.zeros([3, 3])
    sk[0, 1] = -a[2, 0]
    sk[0, 2] = a[1, 0]
    sk[1, 0] = a[2, 0]
    sk[1, 2] = -a[0, 0]
    sk[2, 0] = -a[1, 0]
    sk[2, 1] = a[0, 0]
    return sk


def epsilon_MD(R_MD, R_X, R_TH, t_TD, t_TH, t_X, alpha_TH, alpha_X, epsilon_TD, epsilon_TH, epsilon_X):
    term1 = epsilon_TD
    term2 = R_MD.T@R_X.T@makeSkew(R_TH.T@(t_TD - t_TH))@alpha_TH #??
    term3 = R_MD.T@R_X.T@makeSkew(R_TH.T@(t_TD - t_TH) - t_X)@R_X@alpha_X
    term4 = -R_MD.T@R_X.T@epsilon_TH
    term5 = -R_MD.T@epsilon_X #??
    result = [term1,term2,term3,term4,term5]
    return result, sum(result)#??


def alpha_MD(R_MD, R_TD, R_TH, alpha_TD, alpha_TH, alpha_X):
    return alpha_TD - R_MD.T*alpha_X - R_TD.T*R_TH*alpha_TH


def delta_U(K, epsilon_MD, delta_K, t_MD, Z):
    term1 = (K@epsilon_MD)/Z
    term2 = (delta_K@t_MD)/Z
    res = [term1, term2]
    return res, sum(res)


if __name__ == '__main__':
    epsilon_TD = epsilon_TH = np.array([[0.11], [0], [0]])
    R_MD = R.from_rotvec(np.pi*np.array([0, 0, -1]))
    R_MD = R_MD.as_matrix()
    R_X = R.from_rotvec(np.pi * np.array([0, 0, -1]))
    R_X = R_X.as_matrix()
    R_TH = np.identity(3)

    delta_K = np.zeros([3, 3])
    delta_K[0, 0] = 708
    delta_K[1, 1] = 995

    K = np.identity(3)
    K[0, 0] = 9442
    K[1, 1] = 10103
    K[0, 2] = 540
    K[1, 2] = 960

    cam2marker = np.load('A_cam2marker.npy')
    pose_num = cam2marker.shape[2]
    dis_list = np.zeros([pose_num])
    for i in range(pose_num):
        dis_list[i] = np.linalg.norm(cam2marker[:, :, i][:3, 3])
    Z = np.mean(dis_list)

    X = np.load('hand_eye_X_axxb.npy')
    # print(np.linalg.norm(X[:3, 3]))

    alpha_X = np.array([[11*np.pi/180], [11*np.pi/180], [11*np.pi/180]])
    epsilon_X = np.array([[50], [0], [0]])

    t_TD = np.array([[1000], [0], [0]])
    t_TH = np.array([[1200], [0], [0]]) #???
    t_X = np.array([[170], [0], [0]])
    t_MD = np.array([[250], [0], [0]])

    path = '../geometries/geometryCam_hand.json'
    with open(path, 'r') as load_f:
        load_dict = json.load(load_f)
    points = np.zeros([4, 3])
    points_dis = np.zeros(4)
    for i in range(4):
        points[i, 0] = load_dict['fiducials'][i]['x']
        points[i, 1] = load_dict['fiducials'][i]['y']
        points[i, 2] = load_dict['fiducials'][i]['z']
        points_dis[i] = np.linalg.norm(points[i])
    minDis = np.min(points_dis)
    tmp = math.atan(epsilon_TD[0]/minDis)
    alpha_TH = np.array([[tmp], [tmp], [tmp]])

    # t = epsilon_TD + R_MD.T @ R_X.T @ makeSkew(R_TH.T @ (t_TD - t_TH)) @ alpha_TH

    res_e_MD, epsilon_MD = epsilon_MD(R_MD, R_X, R_TH, t_TD, t_TH, t_X, alpha_TH, alpha_X, epsilon_TD, epsilon_TH, epsilon_X)
    res_delta_U, delta_U = delta_U(K, epsilon_MD, delta_K, t_MD, Z)
    print('epsilon_MD:\n', res_e_MD, epsilon_MD)
    print('delta_U:\n', res_delta_U, delta_U)