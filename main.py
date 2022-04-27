import numpy as np
import pandas as pd
from Solver import solver
from dataLoader import dataLoader
from scipy.spatial.transform import Rotation
import open3d as o3d
import os
import copy
import time
sol = solver()
ld = dataLoader()


def getRegPivot(dirPath):
    fileNum = 0
    p_list = []
    for lst in os.listdir(dirPath):
        sub_path = os.path.join(dirPath, lst)
        # print(sub_path)
        if lst == 'inst':
            for name in os.listdir(sub_path):
                name = os.path.join(os.path.abspath(sub_path), name)
                data = ld.loadJson(name)
                # print(data)
                p_list.append(sol.trackTip(data).T)
                fileNum = fileNum + 1
            p_list = np.vstack(p_list)
    print('point num:', fileNum)
    return p_list

temp = getRegPivot('run_2.21_281_points_cloud')
sol.plotView(temp)
np.save('run_2.21_281_points_cloud/281_points_clouds.npy', temp)

def getPanel(dirPath):
    fileNum = 0
    p_list = []
    for lst in os.listdir(dirPath):
        sub_path = os.path.join(dirPath, lst)
        # print(sub_path)
        if lst == 'hmd':
            for name in os.listdir(sub_path):
                name = os.path.join(os.path.abspath(sub_path), name)
                data = ld.loadJson(name)
                p_list.append(data)
                fileNum = fileNum + 1
            p_list = np.vstack(p_list)
    print('point num:', fileNum)
    return p_list


def npArray2Hdf5(npArray, dataName):
    import h5py
    import cv2
    import os
    import time
    from tqdm import tqdm

    output_dir = "GT_CSV_2.19"
    time_str = time.strftime("%Y%m%d_%H%M%S")
    file = h5py.File(output_dir + '/' + dataName + time_str + ".hdf5", "w")
    data = npArray
    size = int(data.shape[0] / 4)
    # print(size)
    Op2Cam = np.empty([size, 4, 4])
    Cam2Phacon = np.empty([size, 4, 4])
    Cam2Tool = np.empty([size, 4, 4])
    Phacon2Tool = np.empty([size, 4, 4])
    titles = []
    for i in tqdm(range(0, 100), ncols=100, desc="Compressing into HDF5"):
        for i in range(size):
            Op2Cam[i] = data[4*i:4*i + 4, 0:4]
            Cam2Phacon[i] = data[4*i:4*i + 4, 4:8]
            Cam2Tool[i] = data[4*i:4*i + 4, 8:12]
            Phacon2Tool[i] = data[4*i:4*i + 4, 12:]
        # print(Cam2Phacon[0],'\n',Cam2Tool[0],"\n",Phacon2Tool[0])

    file.create_dataset("Atracsys_to_Camera", data=Op2Cam, compression="gzip", chunks=True)
    file.create_dataset("Camera_to_Phacon", data=Cam2Phacon, compression="gzip", chunks=True)
    file.create_dataset("Camera_to_Tool", data=Cam2Tool, compression="gzip", chunks=True)
    file.create_dataset("Phacon_to_Tool", data=Phacon2Tool, compression="gzip", chunks=True)

    file.close()
    print("Finish")

# def getPhacon2Pan(dirPath):
#     res = getPanel(dirPath)
#     res = res.mean(0)
#     F_pan = sol.seven2trans(res)
#     F_trac2p = np.array([[-9.51175402e-01, -3.02971830e-01, 5.89357441e-02, -2.68274579e+01],
#      [-2.54059282e-01, 8.76963644e-01, 4.07907647e-01, -5.18221993e+02],
#      [-1.75269031e-01, 3.73018548e-01,-9.11119055e-01, 1.04849653e+03],
#      [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
#     R_trac2p_inv = F_trac2p[:3, :3].T
#     p_trac2p_inv = -R_trac2p_inv@F_trac2p[:3, 3].reshape(-1, 1)
#     R_p2Rb = R_trac2p_inv@F_pan[0]
#     p_p2Rb = R_trac2p_inv@F_pan[1] + p_trac2p_inv
#     F_p2Rb = np.vstack((np.hstack((R_p2Rb, p_p2Rb)), np.array([0,0,0,1])))
#     print('\n', F_p2Rb)
#     # print(res,'\n', F)
#     return F_p2Rb
#
# # ICP Registration
# def draw_registration_result(source, target, transformation):
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0.706, 0])
#     target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     o3d.visualization.draw_geometries([source_temp, target_temp],
#                                       zoom=0.4459,
#                                       front=[0.9288, -0.2951, -0.2242],
#                                       lookat=[1.6784, 2.0612, 1.4451],
#                                       up=[-0.3402, -0.9189, -0.1996])
#
#
# source = o3d.io.read_point_cloud('soruce_point_cloud_200.pcd')
# # target = o3d.io.read_point_cloud('soruce_point_cloud_200.pcd')
# target = o3d.io.read_point_cloud("phacon.pcd")
# threshold = 7
# # trans_init = result_ransac.transformation
# # trans_init = np.identity(4)
# trans_init = np.asarray([[-9.10805395e-01 ,-3.35944633e-01  ,2.39947360e-01 ,-3.13141290e+02],
#  [-2.04953004e-01  ,8.72487181e-01 , 4.43576808e-01, -5.81168423e+02],
#  [-3.58368243e-01  ,3.54834218e-01 ,-8.63518894e-01  ,1.00066304e+03],
#  [ 0.00000000e+00  ,0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])
#
# # trans_init = np.asarray([[-9.50833429e-01 -3.03908731e-01,  5.96261130e-02 ,-2.75418407e+01],
# #  [-2.54084761e-01 , 8.75564861e-01 , 4.10885760e-01 ,-5.22150360e+02],
# #  [-1.77078299e-01 , 3.75533830e-01, -9.09734917e-01 , 1.04629098e+03],
# #  [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
# for i in range(1):
#     print("Apply point-to-plane ICP")
#     reg_p2l = o3d.pipelines.registration.registration_icp(
#         source, target, threshold, trans_init,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
#     print(reg_p2l)
#     print("Transformation is:")
#     print(reg_p2l.transformation)
#     draw_registration_result(source, target, reg_p2l.transformation)
#     trans_init = reg_p2l.transformation

# path = 'geo_panel_2.8/_slash_atracsys_slash_Controller_slash_measured_cp_array.csv'
# csv_data = pd.read_csv('geo_panel_2.8/_slash_atracsys_slash_Controller_slash_measured_cp_array.csv')
# res = ld.getCentroid(path, 5)
# print(res)
# fs = ld.frames(5, csv_data, panel=True)
# sol.regPanel(csv_data)

# F = getPhacon2Pan('run_2.8_251_points_cloud')

# # pivotTool_geometry:
# pg = np.array(
# [[3.364, -62.125, -1.771],
# [43.56, -18.60, 8.163],
# [-41.23, -15.245, -8.919],
# [-0.9433, 13.187, 0.0272],
# [-4.750,  82.790, 2.499]])
# np.save('pivotTool_geometry', pg)
#
# sol.plotView(pg)

file_num1 = 21
file_num2 = 21
#get final data
pan = getPanel('run_2.21_281_points_cloud')
pan = np.mean(pan, 0)

_, Op2Pan_reg = sol.seven2trans(pan)
# print(Op2Pan)
phacon2Op = np.load('Phacon2Op_2.21.npy')
# phacon2Op_2 = np.load('run_2.8_251_points_cloud/patient2Op(2).npy')
# print(phacon2Op)
phacon2Cam_1 = np.load(f'phacon_camera_registartion_data_2.19/{file_num1}.npy')
phacon2Cam_2 = np.load(f'phacon_camera_registartion_data_2.19/{file_num2}.npy')
# print(phacon2Cam_1)
# print(phacon2Cam_2)
phacon2Pan = phacon2Op@Op2Pan_reg
pan2Phacon = sol.invTransformation(phacon2Pan)

pan2Cam_1 = pan2Phacon@phacon2Cam_1
pan2Cam_2 = pan2Phacon@phacon2Cam_2
# print(pan2Cam_1)
# print(pan2Cam_2)
# cam2Pan_1 = sol.invTransformation(pan2Cam_1)
# cam2Pan_2 = sol.invTransformation(pan2Cam_2)
#12.3190 13.3000 14.2700
inference_idx = {'12': 3190, '13': 3000, '14': 2780, '15': 1520, '16': 2180, '14-2': 2780, '21': 2086}
p_1_tmp = ld.getRealPose(inference_idx[f'{file_num1}'], f'GT_CSV_2.19/ground_truth_data_{file_num1}/fwd_pose_pan.csv')
p_2_tmp = ld.getRealPose(inference_idx[f'{file_num2}'], f'GT_CSV_2.19/ground_truth_data_{file_num2}/fwd_pose_pan.csv')
print(p_1_tmp)
print(p_2_tmp)
# print(p_1_tmp)
_, Op2Pan_data_1 = sol.seven2trans(p_1_tmp)
_, Op2Pan_data_2 = sol.seven2trans(p_2_tmp)
#test on static move
Op2CamHand_10 = ld.getRealPose(0, f'GT_CSV_2.19/ground_truth_data_{file_num1}/fwd_pose_camhand.csv')
Op2CamHand_11 = ld.getRealPose(inference_idx[f'{file_num1}'], f'GT_CSV_2.19/ground_truth_data_{file_num1}/fwd_pose_camhand.csv')

Op2CamHand_20 = ld.getRealPose(0, f'GT_CSV_2.19/ground_truth_data_{file_num2}/fwd_pose_camhand.csv')
Op2CamHand_21 = ld.getRealPose(inference_idx[f'{file_num2}'], f'GT_CSV_2.19/ground_truth_data_{file_num2}/fwd_pose_camhand.csv')

_, Op2CamHand_10 = sol.seven2trans(Op2CamHand_10)
_, Op2CamHand_11 = sol.seven2trans(Op2CamHand_11)
_, Op2CamHand_20 = sol.seven2trans(Op2CamHand_20)
_, Op2CamHand_21 = sol.seven2trans(Op2CamHand_21)

print('data_1 vibration translation:', np.linalg.norm(Op2CamHand_10-Op2CamHand_11))
print('data_1 vibration orientation:\n', Op2CamHand_10[:3, :3] @ np.linalg.inv(Op2CamHand_11[:3, :3]))
print('data_1 vibration transformation:\n', Op2CamHand_10 @ sol.invTransformation(Op2CamHand_11))
# print('data_2 vibration:', np.linalg.norm(Op2CamHand_20-Op2CamHand_21))
# print('data_2 vibration orientation:\n', Op2CamHand_20[:3, :3] @ np.linalg.inv(Op2CamHand_21[:3, :3]))

#################################
Op2Cam_1 = Op2Pan_data_1@pan2Cam_1
Op2Cam_2 = Op2Pan_data_2@pan2Cam_2

res = Op2Cam_1@sol.invTransformation(Op2Cam_2)
dis = np.linalg.norm(Op2Cam_1[:3, 3]) - np.linalg.norm(Op2Cam_2[:3, 3])
Op_Cam_dis1 = np.linalg.norm(Op2Cam_1[:3, 3])
Op_Cam_dis2 = np.linalg.norm(Op2Cam_2[:3, 3])
Op_Camhand_dis1 = np.linalg.norm(Op2CamHand_11[:3, 3])
Op_Camhand_dis2 = np.linalg.norm(Op2CamHand_21[:3, 3])
Op2cam_RinvR = Op2CamHand_11[:3, :3]@Op2CamHand_11[:3, :3].T

print('Trac2Cam_1:\n', Op2Cam_1, '\n', 'Trac2Cam_2:\n', Op2Cam_2)
print('\nOp2Cam_dis1:', Op_Cam_dis1, '\nOp2Cam_dis2:', Op_Cam_dis2)
print('Op_Camhand_dis1:', Op_Camhand_dis1, '\nOp_Camhand_dis2:', Op_Camhand_dis2)
print('Op2cam_RinvR\n', Op2cam_RinvR)
#

def gtCSV2Hdf5(dataIdx):
    drill_path_1 =f'GT_CSV_2.19/ground_truth_data_{dataIdx}/fwd_pose_drill.csv'
    drill_df_1 = pd.read_csv(drill_path_1)
    # print(len(drill_df_1))
    Cam2Op_1 = sol.invTransformation(Op2Cam_1)
    title = ['Tracker to camera', 'Camera to phacon', 'Camera to tool', 'Patient to tool']

    Camera_to_phacon = []
    Camera_to_tool = []
    Patient_to_tool = []
    data = []
    Cam2Op = sol.invTransformation(Op2Cam_1)

    for i in range(len(drill_df_1)):
        Op2Pan = ld.getRealPose(i, f'GT_CSV_2.19/ground_truth_data_{dataIdx}/fwd_pose_pan.csv')
        _, Op2Pan = sol.seven2trans(Op2Pan)
        Camera2phacon = Cam2Op@Op2Pan@pan2Phacon
        # print(Camera2phacon)
        # print("Dis Camera Phacon",np.linalg.norm(Camera2phacon[:3, 3]))
        Op2Drill_1 = ld.getRealPose(i, drill_path_1)
        _, Op2Drill_1 = sol.seven2trans(Op2Drill_1)
        Cam2Drill_1 = Cam2Op_1@Op2Drill_1
        # Patient_to_tool = sol.invTransformation(Camera_to_phacon)@Cam2Drill_1
        Patient2tool = sol.invTransformation(Op2Pan@pan2Phacon)@Op2Drill_1
        Camera_to_phacon.append(Camera2phacon)
        Camera_to_tool.append(Cam2Drill_1)
        Patient_to_tool.append(Patient2tool)
        # break
    data = [np.repeat([Op2Cam_1], len(drill_df_1), axis=0).reshape([4*len(drill_df_1), 4]), np.vstack(Camera_to_phacon),
            np.vstack(Camera_to_tool), np.vstack(Patient_to_tool)]
    data = np.hstack(data)
    np.save('data', data)
    print(data, data.shape)

    npArray2Hdf5(data, f'data_{dataIdx}')


# gtCSV2Hdf5(21)

