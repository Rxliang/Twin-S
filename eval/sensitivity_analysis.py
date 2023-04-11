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


def epsilon_dp(R_pb_p, R_o_pb, R_o_db, R_db_d, R_p_d, t_pb_p, t_o_pb, t_o_db, t_db_d, epsilon_pb_p, epsilon_o_pb, epsilon_db_d, epsilon_o_db, alpha_o_pb, alpha_o_db, alpha_db_d):
    term1 = np.identity(3)@epsilon_pb_p
    term2 = -R_pb_p.T@epsilon_o_pb
    term3 = -R_pb_p.T@R_o_pb.T@R_o_db@R_db_d@epsilon_db_d
    term4 = -R_pb_p.T@R_o_pb.T@R_o_db@epsilon_o_db
    term5 = -makeSkew(R_pb_p@t_pb_p)*R_pb_p.T@alpha_o_pb
    term6 = makeSkew(R_pb_p@t_pb_p + R_pb_p.T@R_o_pb.T@(t_o_pb - t_o_db))@R_pb_p.T@R_o_pb.T@R_o_db@alpha_o_db
    term7 = makeSkew(R_pb_p@t_pb_p + R_pb_p.T@R_o_pb.T@(t_o_pb - t_o_db - R_o_db@t_db_d))@R_p_d@alpha_db_d
    result = [term1, term2, term3, term4, term5, term6, term7]
    return result


def alpha_dp(R_pb_p, R_o_pb, R_o_db, R_d_p, alpha_pb_p, alpha_o_pb, alpha_o_db, alpha_db_d):
    term1 = np.identity(3)@alpha_pb_p
    term2 = R_pb_p.T@alpha_o_pb
    term3 = -R_pb_p.T@R_o_pb.T@R_o_db@alpha_o_db
    term4 = -R_d_p.T@alpha_db_d
    result = [term1, term2, term3, term4]
    return result


def error_dp(R_pb_p, R_o_pb, R_o_db, R_db_d, R_d_p, t_pb_p, t_o_pb, t_o_db, t_db_d, epsilon_pb_p, epsilon_o_pb, epsilon_db_d, epsilon_o_db, alpha_o_pb, alpha_o_db, alpha_db_d, alpha_pb_p):
    ep_dp = epsilon_dp(R_pb_p, R_o_pb, R_o_db, R_db_d, t_pb_p, t_o_pb, t_o_db, t_db_d, epsilon_pb_p, epsilon_o_pb, epsilon_db_d, epsilon_o_db, alpha_o_pb, alpha_o_db, alpha_db_d)
    al_dp = alpha_dp(R_pb_p, R_o_pb, R_o_db, R_d_p, alpha_pb_p, alpha_o_pb, alpha_o_db, alpha_db_d)
    print('epsilon_dp:', ep_dp)
    print('alpha_dp:', al_dp)


def alpha_o(path):
    # path = '../params/geometries/geometryCam_hand.json'
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
    tmp = math.atan(epsilon_TD[0] / minDis)
    alpha = np.array([[tmp], [tmp], [tmp]])
    return alpha


if __name__ == '__main__':
    epsilon_TD = epsilon_TH = np.array([[0.11], [0], [0]])
    R_MD = R.from_rotvec(np.pi*np.array([0, 0, -1]))
    R_MD = R_MD.as_matrix()
    R_X = R.from_rotvec(np.pi * np.array([0, 0, -1]))
    R_X = R_X.as_matrix()
    R_TH = np.identity(3)

    # delta_K = np.zeros([3, 3])
    # delta_K[0, 0] = 708
    # delta_K[1, 1] = 995
    #
    # K = np.identity(3)
    # K[0, 0] = 9442
    # K[1, 1] = 10103
    # K[0, 2] = 540
    # K[1, 2] = 960
    #
    # cam2marker = np.load('A_cam2marker.npy')
    # pose_num = cam2marker.shape[2]
    # dis_list = np.zeros([pose_num])
    # for i in range(pose_num):
    #     dis_list[i] = np.linalg.norm(cam2marker[:, :, i][:3, 3])
    # Z = np.mean(dis_list)
    #
    # X = np.load('hand_eye_X_axxb.npy')
    # # print(np.linalg.norm(X[:3, 3]))
    #
    # alpha_X = np.array([[11*np.pi/180], [11*np.pi/180], [11*np.pi/180]])
    # epsilon_X = np.array([[50], [0], [0]])
    #
    # t_TD = np.array([[1000], [0], [0]])
    # t_TH = np.array([[1200], [0], [0]]) #???
    # t_X = np.array([[170], [0], [0]])
    # t_MD = np.array([[250], [0], [0]])
    #
    # path = '../params/geometries/geometryCam_hand.json'
    # with open(path, 'r') as load_f:
    #     load_dict = json.load(load_f)
    # points = np.zeros([4, 3])
    # points_dis = np.zeros(4)
    # for i in range(4):
    #     points[i, 0] = load_dict['fiducials'][i]['x']
    #     points[i, 1] = load_dict['fiducials'][i]['y']
    #     points[i, 2] = load_dict['fiducials'][i]['z']
    #     points_dis[i] = np.linalg.norm(points[i])
    # minDis = np.min(points_dis)
    # tmp = math.atan(epsilon_TD[0]/minDis)
    # alpha_TH = np.array([[tmp], [tmp], [tmp]])
    # print(alpha_TH)
    #
    # # t = epsilon_TD + R_MD.T @ R_X.T @ makeSkew(R_TH.T @ (t_TD - t_TH)) @ alpha_TH
    #
    # res_e_MD, epsilon_MD = epsilon_MD(R_MD, R_X, R_TH, t_TD, t_TH, t_X, alpha_TH, alpha_X, epsilon_TD, epsilon_TH, epsilon_X)
    # res_delta_U, delta_U = delta_U(K, epsilon_MD, delta_K, t_MD, Z)
    # print('epsilon_MD:\n', res_e_MD, epsilon_MD)
    # print('delta_U:\n', res_delta_U, delta_U)

    # relative error analysis
    R_pb_p = np.identity(3)
    R_o_pb = R_MD
    R_o_db = np.identity(3)
    R_d_p = R_MD
    R_p_d = R_d_p.T
    alpha_pb_p = np.array([[0.2*3.14/180], [0], [0]]) # ICP # important for rotation
    alpha_o_pb = alpha_o('../params/geometries/geometryPan.json')
    alpha_o_db = alpha_o('../params/geometries/geometryDrill.json')
    alpha_db_d = np.array([[0.2*3.14/180], [0.], [0.0]]) # align_vector nan #important for rotation and translation
    res_alpha = alpha_dp(R_pb_p, R_o_pb, R_o_db, R_d_p, alpha_pb_p, alpha_o_pb, alpha_o_db, alpha_db_d)
    print('alpha terms:')
    sum_alpha = 0
    for index, i in enumerate(res_alpha):
        print(f'term {index+1}:', np.linalg.norm(i)*180/3.14, '\n------------')
        sum_alpha += np.linalg.norm(i)*180/3.14
    print('Sum alpha error:', sum_alpha, '\n------------')

    R_pb_p = R_MD
    R_o_pb = np.identity(3)
    R_o_db = np.identity(3)
    R_db_d = np.identity(3)
    t_pb_p = np.array([[-56.1], [-95.6], [3.6]])
    t_o_pb = np.array([[-51.3], [171.1], [1321.3]])
    t_o_db = np.array([[178.0], [6.7], [1064.5]])
    t_db_d = np.array([[12.9], [-171.2], [56.1]])
    epsilon_pb_p = np.array([[0.9], [0], [0]]) # important for translation
    epsilon_o_pb = np.array([[0.08], [0], [0]])
    epsilon_db_d = np.array([[0.03], [0], [0]])
    epsilon_o_db = np.array([[0.08], [0], [0]])
    res_epsilon = epsilon_dp(R_pb_p, R_o_pb, R_o_db, R_db_d, R_p_d, t_pb_p, t_o_pb, t_o_db, t_db_d, epsilon_pb_p, epsilon_o_pb, epsilon_db_d,
               epsilon_o_db, alpha_o_pb, alpha_o_db, alpha_db_d)
    print('epsilon terms:')
    sum_epsilon = 0
    for index, i in enumerate(res_epsilon):
        print(f'term {index+1}:', np.linalg.norm(i),'\n------------')
        sum_epsilon += np.linalg.norm(i)
    print('Sum epsilon error:',sum_epsilon, '\n------------')
