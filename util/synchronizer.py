#!/usr/bin/env python

import os
import rospy
import message_filters
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import PoseStamped
import argparse

count = 0

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
    print('img_sec:',img_sec)

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

def callback_3(limage, rimage, pose_drill, pose_camhand, pose_pan):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    print('img_sec:',img_sec)

    # Publish
    pub1.publish(limage)
    pub2.publish(rimage)
    pub3.publish(pose_drill)
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
    pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
    pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)

    # Publisher
    # pub1 = rospy.Publisher('fwd_pointcloud', PointCloud2, queue_size=50)
    pub1 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
    pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub4 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)

    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_drill_sub, pose_camhand_sub, pose_pan_sub], 50, 0.5)
    ts.registerCallback(callback_3)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################
# Post optimization stable phantom with moving camera

def callback_4(limage, rimage, pose_drill, pose_camhand, pose_pan, pointcloud):
    global count
    # Timestamp info
    img_sec = limage.header.stamp.secs
    print('img_sec:',img_sec)

    # Publish
    pub1.publish(limage)
    pub2.publish(rimage)
    pub3.publish(pose_drill)
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
    pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
    pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)
    pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)
    pointcloud_sub = message_filters.Subscriber('/zedm/zed_node/point_cloud/cloud_registered', PointCloud2)


    # Publisher
    pub1 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
    pub2 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
    pub3 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
    pub4 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)
    pub5 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)
    pub6 = rospy.Publisher('fwd_pointcloud', PointCloud2, queue_size=50)


    ts = message_filters.ApproximateTimeSynchronizer([limage_sub, rimage_sub, pose_drill_sub, pose_camhand_sub, pose_pan_sub, pointcloud_sub], 50, 0.5)
    ts.registerCallback(callback_4)

    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

##########################################################################

def my_shutdown_hook():
    print("in my_shutdown_hook")

def initialization():
    global args
    parser = argparse.ArgumentParser(description="ROS topic synchronizer.")
    parser.add_argument('--data',dest="data", help="Data collection with drill, camera and phantom.", action='store_true')
    parser.add_argument('--eval',dest="eval", help="Evaluate the drill pose projection.", action='store_true')
    parser.add_argument('--post',dest="post", help="Post optimization stable phantom with moving camera.", action='store_true')
    parser.add_argument('--post_depth',dest="post_depth", help="Post optimization stable phantom with moving camera, with ZED pointcloud.", action='store_true')

    args = parser.parse_args()


if __name__ == '__main__':

    initialization()
    if args.data:
        listener_2()
    elif args.eval:
        listener_1()
    elif args.post:
        listener_3()
    elif args.post_depth:
        listener_4()