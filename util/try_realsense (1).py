
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##                  Export to PLY                  ##
#####################################################

# First import the library
import pyrealsense2 as rs
import open3d as o3d
import numpy as np

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

align = rs.align(rs.stream.color)
config = rs.config()
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)

config.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)
pipeline = rs.pipeline()
profile = pipeline.start(config)

depth = pipeline.get_active_profile()
sensor = depth.get_device().query_sensors()[0]  # 0 for depth sensor, 1 for camera sensor
sensor.set_option(rs.option.min_distance, 0)
# sensor.set_option(rs.option.max_distance, 0.5)
sensor.set_option(rs.option.enable_max_usable_range, 0)

intr = profile.get_stream(
    rs.stream.color).as_video_stream_profile().get_intrinsics()
pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
extrinsic = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
post_process = rs.threshold_filter(min_dist = 0, max_dist = 0.7)

def convert_rs_frames_to_pointcloud(rs_frames):
    
    aligned_frames = align.process(rs_frames)

    rs_depth_frame = aligned_frames.get_depth_frame()
    np_depth = np.asanyarray(rs_depth_frame.get_data())
    # print(np_depth.shape)
    rs_depth_frame = post_process.process(rs_depth_frame)

    np_depth = np.asanyarray(rs_depth_frame.get_data())
    # print("After",np_depth.shape)
    o3d_depth = o3d.geometry.Image(np_depth)


    rs_color_frame = aligned_frames.get_color_frame()
    np_color = np.asanyarray(rs_color_frame.get_data())
    o3d_color = o3d.geometry.Image(np_color)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth, depth_scale=4000.0, convert_rgb_to_intensity=False)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, pinhole_camera_intrinsic, extrinsic)

    return pcd

rs_frames = pipeline.wait_for_frames()
pcd = convert_rs_frames_to_pointcloud(rs_frames)
name = "/home/shc/RoboMaster/util/snapshot_5.pcd"
o3d.io.write_point_cloud(name, pcd)
print("Load a ply point cloud, print it, and render it")

pcd = o3d.io.read_point_cloud(name)
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
vis.run()