#!/usr/bin/env python

import os
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import PoseStamped
import argparse

count = 0

##########################################################################
# Data collection with drill, camera and phantom

def callback_VR(pose_drill, pose_pan):
    global count
    # Timestamp info
    img_sec = pose_drill.header.stamp.secs
    spinner_char = spinner[img_sec % len(spinner)]
    print(f"              Synchronizing... {spinner_char}", end="\r")

    # Publish
    pub3.publish(pose_drill)
    pub5.publish(pose_pan)
    count += 1


def listener_VR():
    global pub3, pub5
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)

    # Publisher
    pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)

    ts = message_filters.ApproximateTimeSynchronizer([pose_drill_sub, pose_pan_sub], 50, 0.5)
    ts.registerCallback(callback_VR)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Evaluate the drill pose projection

def callback_1(limage, rimage, pose_drill, pose_camhand):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    print('img_sec:',img_sec)

    # Publish
    pub1.publish(limage)
    pub2.publish(rimage)
    pub3.publish(pose_drill)
    pub4.publish(pose_camhand)
    count += 1


def listener_1():
    global pub1, pub2, pub3, pub4
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    limage_sub = message_filters.Subscriber('/zedm/zed_node/left/image_rect_color/compressed', CompressedImage)
    rimage_sub = message_filters.Subscriber('/zedm/zed_node/right/image_rect_color/compressed', CompressedImage)
    pose_drill_sub = message_filters.Subscriber('/atracsys/Drill/measured_cp', PoseStamped)
    pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)

    # Publisher
    # pub1 = rospy.Publisher('fwd_pointcloud', PointCloud2, queue_size=50)
    pub1 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
    pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub4 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)

    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_drill_sub, pose_camhand_sub], 50, 0.5)
    ts.registerCallback(callback_1)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Data collection with drill, camera and phantom

def callback_2(limage, rimage, pose_drill, pose_camhand, pose_pan):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    spinner_char = spinner[img_sec % len(spinner)]
    print(f"              Synchronizing... {spinner_char}", end="\r")

    # Publish
    pub1.publish(limage)
    pub2.publish(rimage)
    pub3.publish(pose_drill)
    pub4.publish(pose_camhand)
    pub5.publish(pose_pan)
    count += 1


def listener_2():
    global pub1, pub2, pub3, pub4, pub5
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    rimage_sub = message_filters.Subscriber('/zedm/zed_node/right/image_rect_color/compressed', CompressedImage)
    limage_sub = message_filters.Subscriber('/zedm/zed_node/left/image_rect_color/compressed', CompressedImage)
    pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
    pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)

    # Publisher
    pub1 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
    pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub4 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)

    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_drill_sub, pose_camhand_sub, pose_pan_sub], 50, 0.5)
    ts.registerCallback(callback_2)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Post optimization stable phantom with moving camera

def callback_3(limage, rimage, pose_camhand, pose_pan):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    print('img_sec:',img_sec)

    # Publish
    pub1.publish(limage)
    pub2.publish(rimage)
    # pub3.publish(pose_drill)
    pub4.publish(pose_camhand)
    pub5.publish(pose_pan)
    count += 1


def listener_3():
    global pub1, pub2, pub3, pub4, pub5
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    limage_sub = message_filters.Subscriber('/zedm/zed_node/left/image_rect_color/compressed', CompressedImage)
    rimage_sub = message_filters.Subscriber('/zedm/zed_node/right/image_rect_color/compressed', CompressedImage)
    # pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
    pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)

    # Publisher
    # pub1 = rospy.Publisher('fwd_pointcloud', PointCloud2, queue_size=50)
    pub1 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
    # pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub4 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)

    # ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_drill_sub, pose_camhand_sub, pose_pan_sub], 50, 0.5)
    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_camhand_sub, pose_pan_sub], 50, 0.5)

    ts.registerCallback(callback_3)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Post optimization stable phantom with moving camera

def callback_4(limage, rimage, pose_camhand, pose_pan, pointcloud):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    print('img_sec:',img_sec)

    # Publish
    pub1.publish(limage)
    pub2.publish(rimage)
    # pub3.publish(pose_drill)
    pub4.publish(pose_camhand)
    pub5.publish(pose_pan)
    pub6.publish(pointcloud)
    count += 1


def listener_4():
    global pub1, pub2, pub3, pub4, pub5, pub6
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    limage_sub = message_filters.Subscriber('/zedm/zed_node/left/image_rect_color/compressed', CompressedImage)
    rimage_sub = message_filters.Subscriber('/zedm/zed_node/right/image_rect_color/compressed', CompressedImage)
    # pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
    pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)
    pointcloud_sub = message_filters.Subscriber('/zedm/zed_node/point_cloud/cloud_registered', PointCloud2)


    # Publisher
    pub1 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
    # pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub4 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)
    pub6 = rospy.Publisher('fwd_pointcloud', PointCloud2, queue_size=50)


    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_camhand_sub, pose_pan_sub, pointcloud_sub], 50, 0.5)
    ts.registerCallback(callback_4)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Sync recorded images with the sim scene 

def callback_5(rimage, limage, segm, pcd, camhand_pose, pan_pose, drill_pose):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs

    spinner_char = spinner[img_sec % len(spinner)]
    print(f"              Synchronizing... {spinner_char}", end="\r")

    # Publish
    pub0.publish(rimage)
    pub1.publish(limage)
    pub2.publish(segm)
    pub3.publish(pcd)
    pub4.publish(camhand_pose)
    pub5.publish(pan_pose)
    pub6.publish(drill_pose)
    count += 1

def listener_5():
    global pub0, pub1, pub2, pub3, pub4, pub5, pub6
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    limage_sub = message_filters.Subscriber('/pss_limage/compressed', CompressedImage)
    rimage_sub = message_filters.Subscriber('/pss_rimage/compressed', CompressedImage)
    segm_sub = message_filters.Subscriber('/ambf/env/cameras/stereoL/ImageData/compressed', CompressedImage)
    pcd_sub = message_filters.Subscriber('/ambf/env/cameras/main_camera/DepthData', PointCloud2)
    pose_camhand_sub = message_filters.Subscriber('/pss_pose_camhand', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/pss_pose_pan', PoseStamped)
    pose_drill_sub = message_filters.Subscriber('/pss_pose_drill', PoseStamped)

    # Publisher
    pub0 = rospy.Publisher('/sync_rimage/compressed', CompressedImage, queue_size=50)
    pub1 = rospy.Publisher('/sync_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('/sync_segm/compressed', CompressedImage, queue_size=50)
    pub3 = rospy.Publisher('/sync_pcd/DepthData', PointCloud2, queue_size=50)
    pub4 = rospy.Publisher('/sync_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('/sync_pose_pan', PoseStamped, queue_size=50)
    pub6 = rospy.Publisher('/sync_pose_drill', PoseStamped, queue_size=50)

    ts = message_filters.ApproximateTimeSynchronizer([rimage_sub, limage_sub, segm_sub, pcd_sub, pose_camhand_sub, pose_pan_sub, pose_drill_sub], 50, 0.5)
    ts.registerCallback(callback_5)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Sync recorded images with the sim data

def callback_6(limage, segm, pcd, zed_pcd, camhand_pose, pan_pose):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    print('img_sec:',img_sec)

    # Publish
    pub1.publish(limage)
    pub2.publish(segm)
    pub3.publish(pcd)
    pub4.publish(camhand_pose)
    pub5.publish(pan_pose)
    pub6.publish(zed_pcd)
    count += 1

def listener_6():
    global pub1, pub2, pub3, pub4, pub5, pub6
    # Initialize ROS node
    rospy.init_node('image_extract_node', anonymous=True)

    # Subscribers
    limage_sub = message_filters.Subscriber('/pss_limage/compressed', CompressedImage)
    segm_sub = message_filters.Subscriber('/ambf/env/cameras/main_camera/ImageData/compressed', CompressedImage)
    pcd_sub = message_filters.Subscriber('/ambf/env/cameras/main_camera/DepthData', PointCloud2)
    pose_camhand_sub = message_filters.Subscriber('/pss_pose_camhand', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/pss_pose_pan', PoseStamped)
    zed_pcd_sub = message_filters.Subscriber('/pss_pointcloud', PointCloud2)

    # Publisher
    pub1 = rospy.Publisher('/sync_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('/sync_segm/compressed', CompressedImage, queue_size=50)
    pub3 = rospy.Publisher('/sync_pcd/DepthData', PointCloud2, queue_size=50)
    pub4 = rospy.Publisher('/sync_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('/sync_pose_pan', PoseStamped, queue_size=50)
    pub6 = rospy.Publisher('/sync_zedpcd/DepthData', PointCloud2, queue_size=50)

    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, segm_sub, pcd_sub, zed_pcd_sub, pose_camhand_sub, pose_pan_sub], 50, 0.5)
    ts.registerCallback(callback_6)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)
##########################################################################

def my_shutdown_hook():
    print("in my_shutdown_hook")

def initialization():
    global args, spinner
    parser = argparse.ArgumentParser(description="ROS topic synchronizer.")
    parser.add_argument('--vr',dest="vr", help="Only sync drill and phantom poses.", action='store_true')
    parser.add_argument('--data',dest="data", help="Data collection with drill, camera and phantom.", action='store_true')
    parser.add_argument('--eval',dest="eval", help="Evaluate the drill pose projection.", action='store_true')
    parser.add_argument('--post',dest="post", help="Post optimization stable phantom with moving camera.", action='store_true')
    parser.add_argument('--post_depth',dest="post_depth", help="Post optimization stable phantom with moving camera, with ZED pointcloud.", action='store_true')
    parser.add_argument('--sim_sync',dest="sim_sync", help="Sync recorded images with the simulation scene.", action='store_true')
    parser.add_argument('--sync_sim_zed',dest="sync_sim_zed", help="Sync recorded images with the simulation scene and the zed pointcloud.", action='store_true')

    args = parser.parse_args()
    spinner = ['-', '\\', '|', '/']

if __name__ == '__main__':

    initialization()
    if args.vr:
        listener_VR()
    if args.data:
        listener_2()
    elif args.eval:
        listener_1()
    elif args.post:
        listener_3()
    elif args.post_depth:
        listener_4()
    elif args.sim_sync:
        listener_5()
    elif args.sync_sim_zed:
        listener_6()
    