#!/usr/bin/env python

import os

import rospy
import message_filters
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
count = 0

def callback(rimage, limage, pose_pan, pose_drill, pose_camhand):
    global count
    global pub1, pub2, pub3, pub4, pub5
    # print('enter')
    # Timestamp info
    img_sec = rimage.header.stamp.secs
    img_nsec = rimage.header.stamp.nsecs

    # drill_sec = pose_drill.header.stamp.secs
    # drill_nsec = pose_drill.header.stamp.nsecs

    print('img_sec:',img_sec)

    # Publish
    pub1.publish(rimage)
    pub2.publish(limage)
    pub3.publish(pose_pan)
    pub4.publish(pose_drill)
    pub5.publish(pose_camhand)

    count += 1

def my_shutdown_hook():
    print("in my_shutdown_hook")



# Initialize ROS node
rospy.init_node('image_extract_node', anonymous=True)

# Subscribers
rimage_sub = message_filters.Subscriber('/decklink_right/camera/image_raw/compressed', CompressedImage)
limage_sub = message_filters.Subscriber('/decklink_left/camera/image_raw/compressed', CompressedImage)
pose_pan_sub = message_filters.Subscriber('/atracsys/Panel/measured_cp', PoseStamped)
pose_drill_sub = message_filters.Subscriber('/atracsys/Surgical_drill/measured_cp', PoseStamped)
pose_camhand_sub = message_filters.Subscriber('/atracsys/Camera_hand/measured_cp', PoseStamped)


# Publisher
pub1 = rospy.Publisher('fwd_rimage/compressed', CompressedImage, queue_size=50)
pub2 = rospy.Publisher('fwd_limage/compressed', CompressedImage, queue_size=50)
pub3 = rospy.Publisher('fwd_pose_pan', PoseStamped, queue_size=50)
pub4 = rospy.Publisher('fwd_pose_drill', PoseStamped, queue_size=50)
pub5 = rospy.Publisher('fwd_pose_camhand', PoseStamped, queue_size=50)


# while not rospy.is_shutdown():
    
    # Approximate time synchronizing
    # pub1.publish(rimage_sub)
ts = message_filters.ApproximateTimeSynchronizer([rimage_sub, limage_sub, pose_pan_sub, pose_drill_sub, pose_camhand_sub], 50, 0.5)
ts.registerCallback(callback)

rospy.spin()

rospy.on_shutdown(my_shutdown_hook)

#<param name="gscam_config" value="decklinkvideosrc device-number=0 mode=1080i5994 connection=sdi ! deinterlace method = scalerbob ! videoconvert "/>