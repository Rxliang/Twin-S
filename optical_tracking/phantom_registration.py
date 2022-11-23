import sys
sys.path.append('../')
sys.path.insert(0, '/home/shc/RoboMaster/util')
import numpy as np
import copy
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataLoader import dataLoader
from Solver import solver
import os
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
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(source, target, voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    # trans = np.load('../params/Pan2phacon.npy')
    # print(trans_init)
    trans_init = np.identity(4)
    trans_init = np.array([[-0.16558129 ,-0.72285446 , 0.6708683  ,-0.83473582],
 [-0.29884375 , 0.68505143 , 0.66437712 ,-0.76449373],
 [-0.93982724 ,-0.09047638 ,-0.32945224 , 0.3722269 ],
 [ 0.     ,     0.     ,     0.      ,    1.        ]])

    # source.transform(trans_init)
    draw_registration_result(source, target, trans_init)
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def demo_crop_geometry():
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    pcd_data = o3d.data.DemoICPPointClouds()
    pcd = o3d.io.read_point_cloud(pcd_data.paths[0])
    o3d.visualization.draw_geometries_with_editing([pcd])


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
    op2pan_array = ld.loadMaualPointCloud(dirpath, 'hmd')
    op2drill_array = ld.loadMaualPointCloud(dirpath, 'inst')
    _, op2pan = sol.seven2trans(op2pan_array[10])
    t_tip = np.array([12.83037253, -168.75504173, 56.3511996 ])/1000

    for i in range(len(op2drill_array)):
        point = sol.trackTip(op2drill_array[i], t_tip).T
        point_cloud.append(point)
    point_cloud = np.vstack(point_cloud)
    # visulizePoints(point_cloud)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    o3d.io.write_point_cloud("../data/phantom_point-cloud_data/phacon_1017_388_in_meters.ply", pcd)
    pcd_source = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/phacon_1017_388_in_meters.ply")

    # # # convert mesh to point cloud and save
    mesh = o3d.io.read_triangle_mesh("../data/phantom_point-cloud_data/phacon_1028.stl")
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = mesh.vertices
    pcd2.colors = mesh.vertex_colors
    pcd2.normals = mesh.vertex_normals
    o3d.io.write_point_cloud("../data/phantom_point-cloud_data/phacon_1028.ply", pcd2)

    pcd_target = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/phacon_1028.ply")
    pcd_target = pcd_target.voxel_down_sample(voxel_size=0.001)
    # o3d.visualization.draw_geometries([pcd_target, pcd_source])
    
    voxel_size=0.05

    result = manual_registration(pcd_source, pcd_target)

    print("Apply point-to-plane ICP")

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd_source)
    # vis.add_geometry(pcd_target)
    threshold = 0.0008
    # icp_iteration = 10
    
    trans_init_icp = result.transformation
    # for i in range(icp_iteration):
    reg_p2l = o3d.pipelines.registration.registration_icp(
        pcd_source, pcd_target, threshold, trans_init_icp,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    # pcd_source.transform(reg_p2l.transformation)
    #     vis.update_geometry(pcd_source)
    #     vis.poll_events()
    #     vis.update_renderer()

    # vis.destroy_window()
    # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(pcd_source, pcd_target, reg_p2l.transformation)
    ph2op = reg_p2l.transformation
    # print(sol.invTransformation(ph2op))
    ph2pan = ph2op @ op2pan
    return ph2pan
    # draw_registration_result(source, target, np.identity(4))


def colorbar(norm):
    fig, ax = plt.subplots(figsize=(6, 6))

    fig.subplots_adjust(bottom=0.5)

    cbar = fig.colorbar(
        mpl.cm.ScalarMappable(norm=norm, cmap='jet'),
        cax=ax,
        orientation="vertical")
    cbar.ax.tick_params(labelsize=20)
    cbar.set_label('mm', fontsize=20)
    plt.show()


if __name__ == '__main__':
    dirpath = sys.argv[1]
    ph2pan = phantom_registration(dirpath)
    # ph2pan = np.load('../params/phacon2pan.npy')
    ph2pan[:3,3] = ph2pan[:3,3]*1000
    # print(ph2pan)
    # np.save('../params/phacon2pan_1028.npy', ph2pan)

    cmap_norm = mpl.colors.Normalize(vmin=5.884951730050053e-03, vmax=2.867834215281021)
    colorbar(cmap_norm)