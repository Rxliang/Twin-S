import sys
sys.path.insert(0, '/home/shc/Twin-S/util')
from dataLoader import dataLoader
from Solver import solver

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as Rot


sol = solver()
ld = dataLoader()

def getNdarray(pose_path):
    csv_data = pd.read_csv(pose_path)
    # print(csv_data.head())
    num_frames = len(csv_data)
    translation_array = []
    pivot = np.load('/home/shc/Twin-S/params/t_tip.npy')
    for i in range(num_frames):
        # quaternion = ld.getToolPose(i, csv_data)
        seven_params = ld.getToolPose(i, pose_path)
        _, F = sol.seven2trans(seven_params)
        translation_array.append((F[:3, :3]@pivot).T + F[:3, 3])
    translation_array = np.vstack(translation_array)
    return translation_array

def getNorm(translation_array):
    num_frames = len(translation_array)
    dislist = []
    for i in range(num_frames):
        try:
            dis = np.linalg.norm(translation_array[i+1]-translation_array[i])
            dislist.append(dis)
        except:
            dislist = np.vstack(dislist)
            return dislist

def visulizePoints(point_array):
    x = point_array[:, 0]
    y = point_array[:, 1]
    z = point_array[:, 2]

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z)
    
    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()

def axisCalibration(pose_path, translation_array):
    dislist = getNorm(translation_array)
    pointsInDrill = np.zeros([len(translation_array), 3])
    
    for i in range(len(dislist)):
        pointsInDrill[i+1, 0] = np.sum(dislist[:i+1])
    # return
    # scipy vector align to get the rotation
    (r, rmsd, sensi) = Rot.align_vectors(pointsInDrill, translation_array, return_sensitivity=True)
    R_o_tip = r.as_matrix()
    # R = r.as_euler('zyx', degrees=True)
    print('ss:',sensi)

    csv_data = pd.read_csv(pose_path)
    num_frames = len(csv_data)
    translation_array = []
    # pivot = np.array([[12.83037253, -168.75504173, 56.3511996]]).T ##17
    pivot = np.load('/home/shc/Twin-S/params/t_tip.npy')
    for i in range(num_frames):
        # quaternion = ld.getToolPose(i, csv_data)
        seven_params = ld.getToolPose(i, pose_path)
        _, F_o_drill = sol.seven2trans(seven_params)
        R_d_tip = sol.invTransformation(F_o_drill)[:3, :3]@R_o_tip
        r = Rot.from_matrix(R_d_tip)
        euler = r.as_euler('zyx', degrees=True)
        # print(R_d_tip)
    # print(R_d_tip)
    # np.save('../params/R_d_tip_1.npy', R_d_tip)

if __name__ == '__main__':
    pose_path = sys.argv[1]
    translation_array = getNdarray(pose_path)
    ## oppsite motion
    translation_array = translation_array[::-1,:]
    dislist = getNorm(translation_array)
    visulizePoints(translation_array)
    axisCalibration(pose_path, translation_array)