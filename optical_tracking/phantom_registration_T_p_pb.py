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
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
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


def phantom_registration(dirpath):
    point_cloud = []
    T_o_pb_array = ld.loadMaualPointCloud(dirpath, 'base')
    T_o_t_array = ld.loadMaualPointCloud(dirpath, 'tool')
    
    t_tip = np.load('../params/pointer_tip_0410.npy') / 1000
    print(t_tip)
    for i in range(len(T_o_t_array)):
        _, T_o_pb = sol.seven2trans(T_o_pb_array[i])
        _, T_o_t = sol.seven2trans(T_o_t_array[i])
        T_pb_o = sol.invTransformation(T_o_pb)
        T_pb_t = T_pb_o@T_o_t
        print(np.linalg.norm(T_pb_t[:3,3]))
        point = T_pb_t[:3, :3]@t_tip + T_pb_t[:3, 3, None]
        point = point.reshape(1,-1)
        # point = sol.trackTip(T_o_t_array[i], t_tip).T
        point_cloud.append(point)
    point_cloud = np.vstack(point_cloud)
    # print(point_cloud)
    # visulizePoints(point_cloud)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd_source = pcd
    # pcd_source = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/sampled_0410.ply")
    # pcd_target = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/phacon_exp_3.ply")
    # pcd_target = pcd_target.voxel_down_sample(voxel_size=0.001)
    pcd_target = o3d.io.read_point_cloud('/home/shc/Documents/phacon_data/phacon_0314/phacon_0314_ds.ply')
    # o3d.visualization.draw_geometries([pcd_target, pcd_source])
    
    voxel_size=0.05

    result = manual_registration(pcd_source, pcd_target)

    print("Apply point-to-plane ICP")

    threshold = 0.008
    # icp_iteration = 10
    loss = o3d.pipelines.registration.TukeyLoss(k=0.005)
    trans_init_icp = result.transformation

    reg_p2l = o3d.pipelines.registration.registration_icp(
        pcd_source, pcd_target, threshold, trans_init_icp,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss))

    

    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(pcd_source, pcd_target, reg_p2l.transformation)
    T_p_pb = reg_p2l.transformation
    # print(sol.invTransformation(ph2op))
    return T_p_pb, reg_p2l
    # draw_registration_result(source, target, np.identity(4))




if __name__ == '__main__':

    path = '../data/phantom_point-cloud_data/sampled_pointcloud/'
    dirpath = os.path.join(path, sys.argv[1])
    result, reg_p2p = phantom_registration(dirpath)
    # ph2pan = np.load('../params/phacon2pan.npy')
    T_p_pb = result.copy()
    T_p_pb[:3,3] = T_p_pb[:3,3]*1000

    print('Pan to phacon:', np.linalg.norm(T_p_pb[:3, 3]))
    np.save('../params/phacon2pan_4.npy', T_p_pb)
