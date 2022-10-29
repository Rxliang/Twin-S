#!/usr/bin/env python

import os

import rospy
import message_filters
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import PoseStamped
count = 0

def callback(limage, segm, depth):
    global count
    global pub1, pub2, pub3, pub4, pub5, pub6, pub7
    # print('enter')
    # Timestamp info
    ambf_stamp = segm.header.stamp
    limage.header.stamp = ambf_stamp
    # drill_sec = pose_drill.header.stamp.secs
    # drill_nsec = pose_drill.header.stamp.nsecs

    print('img_sec:',limage.header.stamp)

    # Publish
    # pub1.publish(rimage)
    pub2.publish(limage)
    # pub3.publish(pose_pan)
    # pub4.publish(pose_drill)
    # pub5.publish(pose_camhand)
    pub6.publish(segm)
    pub7.publish(depth)

    count += 1


def my_shutdown_hook():
    print("in my_shutdown_hook")


 
# Initialize ROS node
rospy.init_node('image_extract_node', anonymous=True)

# Subscribers
# rimage_sub = message_filters.Subscriber('/zedm/zed_node/right/image_rect_color/compressed', CompressedImage)
limage_sub = message_filters.Subscriber('ffwd_limage/compressed', CompressedImage)
segm_sub = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/ImageData/compressed', CompressedImage)
depth_sub = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/DepthData', PointCloud2)
# pose_pan_sub = message_filters.Subscriber('fwd_pose_pan', PoseStamped)
# pose_drill_sub = message_filters.Subscriber('fwd_pose_drill', PoseStamped)
# pose_camhand_sub = message_filters.Subscriber('fwd_pose_camhand', PoseStamped)


# Publisher
# pub1 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
pub2 = rospy.Publisher('sync_limage/compressed', CompressedImage, queue_size=50)
# pub3 = rospy.Publisher('sync_pose_pan', PoseStamped, queue_size=50)
# pub4 = rospy.Publisher('sync_pose_drill', PoseStamped, queue_size=50)
# pub5 = rospy.Publisher('sync_pose_camhand', PoseStamped, queue_size=50)
pub6 = rospy.Publisher('sync_segm/compressed', CompressedImage, queue_size=50)
pub7 = rospy.Publisher('sync_depthData', PointCloud2, queue_size=50)

ts = message_filters.ApproximateTimeSynchronizer([limage_sub, segm_sub, depth_sub], 50, 0.5)
# test = message_filters.ApproximateTimeSynchronizer([rimage_sub, segm_sub], 50, 0.5)
ts.registerCallback(callback)

rospy.spin()
rospy.on_shutdown(my_shutdown_hook)

# /fwd_rimage/compressed /fwd_limage/compressed /ambf/env/cameras/segmentation_camera/ImageData/compressed /ambf/env/cameras/segmentation_camera/DepthData /fwd_pose_pan /fwd_pose_drill /fwd_pose_camhand

# /zedm/zed_node/right/image_rect_color/compressed /zedm/zed_node/left/image_rect_color/compressed /ambf/env/cameras/segmentation_camera/ImageData/compressed /ambf/env/cameras/segmentation_camera/DepthData /atracsys/Panel/measured_cp /atracsys/Surgical_drill/measured_cp /atracsys/Camera_hand/measured_cp