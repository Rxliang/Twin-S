import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import PointCloud2,PointField
import copy
import ros_numpy


align = rs.align(rs.stream.color)
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

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


def o3dpc_to_rospc(o3dpc, frame_id=None, stamp=None):
    """ convert open3d point cloud to ros point cloud
    Args:
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        frame_id (string): frame id of ros point cloud header
        stamp (rospy.Time): time stamp of ros point cloud header
    Returns:
        rospc (sensor.msg.PointCloud2): ros point cloud message
    """
    BIT_MOVE_16 = 2**16
    BIT_MOVE_8 = 2**8
    cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
    is_color = o3dpc.colors
        

    n_points = len(cloud_npy[:, 0])
    if is_color:
        data = np.zeros(n_points, dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('rgb', np.uint32)
        ])
    else:
        data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
            ])
    data['x'] = cloud_npy[:, 0]
    data['y'] = cloud_npy[:, 1]
    data['z'] = cloud_npy[:, 2]
    
    if is_color:
        rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
        rgb_npy = np.floor(rgb_npy*255) # nx3 matrix
        rgb_npy = rgb_npy[:, 0] * BIT_MOVE_16 + rgb_npy[:, 1] * BIT_MOVE_8 + rgb_npy[:, 2]  
        rgb_npy = rgb_npy.astype(np.uint32)
        data['rgb'] = rgb_npy

    rospc = ros_numpy.msgify(PointCloud2, data)
    
    if frame_id is not None:
        rospc.header.frame_id = frame_id
    
    if stamp is None:
        rospc.header.stamp = rospy.Time.now()
    else:
        rospc.header.stamp = stamp
    rospc.height = 1
    rospc.width = n_points
    rospc.fields = []
    rospc.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))    

    if is_color:
        rospc.fields.append(PointField(
                        name="rgb",
                        offset=12,
                        datatype=PointField.UINT32, count=1))    
        rospc.point_step = 16
    else:
        rospc.point_step = 12
    
    rospc.is_bigendian = False
    rospc.row_step = rospc.point_step * n_points
    rospc.is_dense = True
    return rospc

def main_visualize():

    rs_frames = pipeline.wait_for_frames()
    
    pcd = convert_rs_frames_to_pointcloud(rs_frames)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud Visualizer",
                      width=400, height=400)
    vis.add_geometry(pcd)
    render_opt = vis.get_render_option()
    render_opt.point_size = 1.0

    while True:
        rs_frames = pipeline.wait_for_frames()
        pcd_new = convert_rs_frames_to_pointcloud(rs_frames)
        pcd.points = pcd_new.points
        pcd.colors = pcd_new.colors
        vis.update_geometry(pcd)
        if vis.poll_events():
            vis.update_renderer()
        else:
            break

    vis.destroy_window()
    pipeline.stop()

def main_pub():
    rs_frames = pipeline.wait_for_frames()
    pcd = convert_rs_frames_to_pointcloud(rs_frames)

    i ='real'
    # decode pcd to points data to be stored
    pointcloud_msg = o3dpc_to_rospc(pcd,i,stamp = rospy.Time().now())
    pub.publish(pointcloud_msg)
    
    while not rospy.is_shutdown():
        rs_frames = pipeline.wait_for_frames()
        pcd_new = convert_rs_frames_to_pointcloud(rs_frames)
        pointcloud_msg = o3dpc_to_rospc(pcd_new,i,stamp = rospy.Time().now())

        pub.publish(pointcloud_msg)
        print(pointcloud_msg.header.stamp)
        # print("publish...")
        rate.sleep()
    return 0


if __name__ == "__main__":
    pub = rospy.Publisher('realsense_pcl', PointCloud2, queue_size=10)
    rospy.init_node('pointcloud_publisher_node', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    
    # main_pub()
    # rospy.spin()
    # rospy.on_shutdown(my_shutdown_hook)
    main_visualize()
