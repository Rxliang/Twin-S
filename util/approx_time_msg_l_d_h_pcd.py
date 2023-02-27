#!/usr/bin/env python

import os

import rospy
import message_filters
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import PoseStamped
count = 0

def callback(limage, pointcloud, pointcloud_sim, pose_drill, pose_camhand):
# def callback(limage, pose_camhand):

    global count
    global pub1, pub2, pub3, pub4, pub5
    # print('enter')
    # Timestamp info
    img_sec = limage.header.stamp.secs
    img_nsec = limage.header.stamp.nsecs

    # drill_sec = pose_drill.header.stamp.secs
    # drill_nsec = pose_drill.header.stamp.nsecs

    print('img_sec:',img_sec)

    # Publish
    pub1.publish(pointcloud)
    pub2.publish(limage)
    pub3.publish(pointcloud_sim)
    pub4.publish(pose_drill)
    pub5.publish(pose_camhand)

    count += 1

def my_shutdown_hook():
    print("in my_shutdown_hook")



# Initialize ROS node
rospy.init_node('image_extract_node', anonymous=True)

# Subscribers
pointcloud_sub = message_filters.Subscriber('/zedm/zed_node/point_cloud/cloud_registered', PointCloud2) #
limage_sub = message_filters.Subscriber('/zedm/zed_node/left/image_rect_color/compressed', CompressedImage)
pointcloud_sim_sub = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/DepthData', PointCloud2) #
pose_drill_sub = message_filters.Subscriber('/atracsys/Drill/measured_cp', PoseStamped)
pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)


# Publisher
pub1 = rospy.Publisher('fwd_pointcloud', PointCloud2, queue_size=50)
pub2 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
pub3 = rospy.Publisher('fwd_sim_pointcloud', PointCloud2, queue_size=50)
pub4 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
pub5 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)


# while not rospy.is_shutdown():
    
    # Approximate time synchronizing
    # pub1.publish(rimage_sub)
ts = message_filters.ApproximateTimeSynchronizer([limage_sub, pointcloud_sub, pointcloud_sim_sub, pose_drill_sub, pose_camhand_sub], 50, 0.5)
# ts = message_filters.ApproximateTimeSynchronizer([limage_sub, pose_camhand_sub], 50, 0.5)
ts.registerCallback(callback)

rospy.spin()

rospy.on_shutdown(my_shutdown_hook)

#<param name="gscam_config" value="decklinkvideosrc device-number=0 mode=1080i5994 connection=sdi ! deinterlace method = scalerbob ! videoconvert "/>
