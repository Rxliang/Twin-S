import sys
sys.path.append('../')
sys.path.insert(0, '/home/shc/Twin-S/util')
import numpy as np
import copy
import copy
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from dataLoader import dataLoader
from Solver import solver
import os
import seaborn as sns

ld = dataLoader()
sol = solver()

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


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def pick_points(pcd):
    # print("")
    # print(
    #     "1) Please pick at least three correspondences using [shift + left click]"
    # )
    # print("   Press [shift + right click] to undo point picking")
    # print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def manual_registration(source, target):

    print("Visualization of two point clouds before manual alignment")
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    for idx in picked_id_source:
        point = source.points[idx]
        # print(np.linalg.norm(point))
        print(point)


    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    print("")
    return reg_p2p

def PointsRegistration(source_points, target_points):
    # Create Open3D point cloud objects
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_points)
    target_pcd.points = o3d.utility.Vector3dVector(target_points)

    # # Perform 3-point registration
    # transformation = o3d.pipelines.registration.registration_icp(
    #     source_pcd, target_pcd, o3d.pipelines.registration.TransformationEstimationPointToPoint())
    res = manual_registration(source_pcd, target_pcd)
    # Get the aligned source point cloud
    aligned_source_pcd = source_pcd.transform(res.transformation)

    # # Print the transformation matrix
    # print("Transformation matrix:")
    # print(res.transformation)

    # Visualize the point clouds
    o3d.visualization.draw_geometries([target_pcd, aligned_source_pcd])
    print(res)
    s_p = np.asarray(aligned_source_pcd.points)
    t_p = np.asarray(target_pcd.points)
    distances = np.linalg.norm(s_p - t_p, axis=1)
    mean_distance = np.mean(distances)
    print('mean_distance: ', mean_distance)

if __name__ == '__main__':
    tip_pose = np.load('../util/tip_pose.npy')
    t1 = np.mean(tip_pose[:3, :], 0)
    t2 = np.mean(tip_pose[3:6, :], 0)
    t3 = np.mean(tip_pose[6:, :], 0)
    cam_tip = np.vstack([t1, t2, t3])
    # print(np.linalg.norm(cam_tip, axis=1))
    print(cam_tip)
    data_path = '/home/shc/Documents/Twin-S test data/'
    pcd_target = o3d.io.read_point_cloud(data_path + "ct_3.ply")
    pcd_source = o3d.io.read_point_cloud(data_path + "ambf.ply")
    
    R_zed2cv = np.array([[0,1,0],[0,0,-1],[-1,0,0]])
    pcd_source.points = o3d.utility.Vector3dVector((R_zed2cv@np.asarray(pcd_source.points).transpose(-1,-2)).transpose(-1,-2))

    
    # reg_p2p = manual_registration(pcd_source, pcd_target)
    # print(reg_p2p)
    # reg_res = reg_p2p.transformation
    # np.save('reg_res.npy', reg_p2p.transformation)
    # reg_res = np.load(data_path + 'reg_res.npy')
    # inv_reg = sol.invTransformation(reg_res)
    # pcd_target.transform(inv_reg)
    picked_id_target = pick_points(pcd_target)
    target_pickpoints = []
    for idx in picked_id_target:
        point = pcd_target.points[idx]
        target_pickpoints.append(point)
    target_pickpoints = np.vstack(target_pickpoints)

    # t1 = np.mean(target_pickpoints[:3, :], 0)
    # t2 = np.mean(target_pickpoints[3:6, :], 0)
    # t3 = np.mean(target_pickpoints[6:, :], 0)
    # target_pickpoints = np.vstack([t1, t2, t3])

    # print(target_pickpoints)
    # distances = np.linalg.norm(cam_tip - target_pickpoints, axis=1)
    # mean_distance = np.mean(distances)
    # print(mean_distance)
    # squared_distances = np.asarray(reg_p2p.inlier_residuals)
    # mean_squared_distance = np.mean(squared_distances)
    # tre = np.sqrt(mean_squared_distance)
    # print(tre)

    PointsRegistration(cam_tip, target_pickpoints)