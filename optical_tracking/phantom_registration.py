import sys
sys.path.append('../')
sys.path.insert(0, '/home/shc/RoboMaster/util')
import numpy as np
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

def phantom_registration(dirpath, pcd_source, pcd_target):
    # point_cloud = []
    op2pan_array = ld.loadMaualPointCloud(dirpath, 'hmd')
    op2drill_array = ld.loadMaualPointCloud(dirpath, 'inst')
    _, op2pan = sol.seven2trans(op2pan_array[10])
    # t_tip = np.array([12.83037253, -168.75504173, 56.3511996 ])/1000

    # for i in range(len(op2drill_array)):
    #     point = sol.trackTip(op2drill_array[i], t_tip).T
    #     point_cloud.append(point)
    # point_cloud = np.vstack(point_cloud)
    # # visulizePoints(point_cloud)

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(point_cloud)
    # # o3d.io.write_point_cloud("../data/phantom_point-cloud_data/phacon_1017_388_in_meters.ply", pcd)
    # # pcd_source = o3d.io.read_point_cloud("../data/phantom_point-cloud_data/phacon_1017_388_in_meters.ply")
    # pcd_source = o3d.io.read_point_cloud("../util/cropped_exp_3.ply")


    # # # # convert mesh to point cloud and save
    # mesh = o3d.io.read_triangle_mesh("/home/shc/Documents/phacon_data/phacon_exp_3.stl")
    # pcd2 = o3d.geometry.PointCloud()
    # pcd2.points = mesh.vertices
    # pcd2.colors = mesh.vertex_colors
    # pcd2.normals = mesh.vertex_normals
    # pcd_path = '../data/phantom_point-cloud_data/phacon_exp_2.ply'
    # o3d.io.write_point_cloud(pcd_path, pcd2)

    # pcd_target = o3d.io.read_point_cloud(pcd_path)
    # pcd_target = pcd_target.voxel_down_sample(voxel_size=0.001)
    
    voxel_size=0.05

    result = manual_registration(pcd_source, pcd_target)

    print("Apply point-to-plane ICP")

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd_source)
    # vis.add_geometry(pcd_target)
    threshold = 0.005
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
    # index = np.asarray(reg_p2l.correspondence_set)
    # pcd.points[index [0,:5]]
    draw_registration_result(pcd_source, pcd_target, reg_p2l.transformation)
    ph2op = reg_p2l.transformation
    np.save('reg_trans_exp_3', reg_p2l.transformation)
    np.save('reg_corres_set_exp_3', reg_p2l.correspondence_set)

    # print(sol.invTransformation(ph2op))
    ph2pan =ph2op @ op2pan
    return ph2pan, reg_p2l


def eval_registration(reg_p2l, pcd_source, pcd_target):
    pcd_source_tmp = copy.deepcopy(pcd_source)
    # correspondence_set = np.load('reg_corres_set_exp_3.npy')
    # reg_trans = np.load('reg_trans_exp_3.npy')
    correspondence_set = np.asarray((reg_p2l.correspondence_set))
    reg_trans = reg_p2l.transformation
    # correspondence_set = np.load('eval_corr.npy')
    print(correspondence_set.shape)
    source_corr = np.asarray(pcd_source_tmp.transform(reg_trans).points)[correspondence_set[:,0],:]


    target_corr = np.asarray(pcd_target.points)[correspondence_set[:,1],:]
    print((source_corr-target_corr).shape)

    dist_pointwise = np.linalg.norm(source_corr-target_corr,axis = 1)
    print(dist_pointwise.shape)
    
    ## get the error of drilling area
    # tmp = np.array([dist_pointwise > 0.00083]).ravel()
    # c_points = dist_pointwise[tmp]
    # print(c_points.shape)
    # plt.subplot(1,2,1)
    # sns.distplot(dist_pointwise, bins=40, hist=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"},axlabel='m')
    # plt.subplot(1,2,2)
    # sns.distplot(c_points, bins=40, hist=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"},axlabel='m')
    # plt.show()
    # print(np.mean(c_points), np.std(c_points))

    print(np.min(dist_pointwise),np.max(dist_pointwise))
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(source_corr)
    cmap_norm = mpl.colors.Normalize(vmin=np.min(dist_pointwise), vmax=0.0028)#np.max(dist_pointwise))
    point_colors = plt.get_cmap('jet')(cmap_norm(dist_pointwise))[:, 0:3]
    new_pcd.colors = o3d.utility.Vector3dVector(point_colors)
    print(new_pcd.has_colors())
    # pcd_source_tmp.colors = o3d.utility.Vector3dVector(np.ones((73162,3)))
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(new_pcd)
    vis.run()
    return cmap_norm
    # draw_registration_result(source, target, np.identity(4))

def colorbar(norm):

    fig, ax = plt.subplots(figsize=(6,6))

    fig.subplots_adjust(bottom=0.5)
    
    cbar = fig.colorbar(
        mpl.cm.ScalarMappable(norm=norm,cmap='jet'),
        cax=ax,
        orientation="vertical")
    cbar.ax.tick_params(labelsize=20)
    cbar.set_label('mm', fontsize=20)
    plt.show()


if __name__ == '__main__':
    dirpath = sys.argv[1]
    # ph2pan, reg_p2p = phantom_registration(dirpath)
    # ph2pan = np.load('../params/phacon2pan.npy')
    # ph2pan[:3,3] = ph2pan[:3,3]*1000
    # print(ph2pan)
    # np.save('../params/phacon2pan_1028.npy', ph2pan)

    pcd_source = o3d.io.read_point_cloud("../util/cropped_exp_3.ply")
    
    # pcd_source = o3d.io.read_point_cloud("../util/cropped_exp_3_test.ply")
     # # # convert mesh to point cloud and save
    mesh = o3d.io.read_triangle_mesh("/home/shc/Documents/phacon_data/phacon_exp_3.stl")
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = mesh.vertices
    pcd2.colors = mesh.vertex_colors
    pcd2.normals = mesh.vertex_normals
    pcd_path = '../data/phantom_point-cloud_data/phacon_exp_3.ply'
    o3d.io.write_point_cloud(pcd_path, pcd2)
    pcd_target = o3d.io.read_point_cloud(pcd_path)
    # o3d.visualization.draw_geometries([pcd_target])
    # ph2pan, reg_p2p = phantom_registration(dirpath, pcd_source, pcd_target)
    # cmap_norm = eval_registration(reg_p2p, pcd_source, pcd_target)
    cmap_norm = mpl.colors.Normalize(vmin=5.884951730050053e-03, vmax=2.867834215281021)
    colorbar(cmap_norm)
    
    # 2.3181307950055933e-06 0.002865381950155596
    # 5.9692170177387505e-06 0.0024547775807794805