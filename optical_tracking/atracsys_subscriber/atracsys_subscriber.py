from __future__ import with_statement
from __future__ import absolute_import
import threading
import cv2
import os
import json
import yaml

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from io import open

bridge = CvBridge()


OUTPUT_PATH = u'../../data/phantom_point-cloud_data/sampled_pointcloud'

###############################################################################

# Global variables
img_save_counter = 0

hmd_lock = threading.Lock()
inst_lock = threading.Lock()
limg_lock = threading.Lock()
rimg_lock = threading.Lock()

hmd_msg = u""
inst_msg = u""
limg_msg = CompressedImage()
rimg_msg = CompressedImage()

###############################################################################

# create a callback and listener (and lock) for each ros topic

def hmd_callback(msg):
    global run_save_path, img_save_counter, hmd_msg, hmd_lock
    # print(u'msg',msg)
    with hmd_lock:
        # print("In Lock")
        hmd_msg = msg

def hmd_listener():
    rospy.Subscriber(u"/atracsys/Panel/measured_cp", PoseStamped, hmd_callback)
    rospy.spin()

def inst_callback(msg):
    global run_save_path, img_save_counter, inst_msg, inst_lock
    # print(u'msg',msg)
    with inst_lock:
        inst_msg = msg

def inst_listener():
    rospy.Subscriber(u"/atracsys/Surgical_drill/measured_cp", PoseStamped, inst_callback)
    rospy.spin()

#callback and listener for img topics
# def limg_callback(msg):
#     global run_save_path, img_save_counter, limg_msg, limg_lock
#     # print(u'msg',msg)
#     with limg_lock:
#         limg_msg = msg

# def limg_listener():
#     rospy.Subscriber(u"/zedm/zed_node/left/image_rect_color/compressed", CompressedImage, limg_callback)
#     rospy.spin()

# def rimg_callback(msg):
#     global run_save_path, img_save_counter, rimg_msg, rimg_lock
#     # print(u'msg',msg)
#     with rimg_lock:
#         rimg_msg = msg

# def rimg_listener():
#     rospy.Subscriber(u"/zedm/zed_node/right/image_rect_color/compressed", CompressedImage, rimg_callback)
#     rospy.spin()


def save_ros_msg_as_json(msg, path):
   # Save a string ROS message to JSON format
    y = yaml.safe_load(str(msg))
    # print(y)
    with open( path, u"w") as f:
        json.dump(y, f, indent=4)
    
# def save_ros_msg_as_img(msg, path):
#     cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
#     cv2.imwrite(path, cv_img)

def save_all_ros_msg():
    global img_save_counter

    with hmd_lock:
        save_ros_msg_as_json(hmd_msg, os.path.join(run_save_path, "HMD_POSE"+str(img_save_counter)+".json"))

    with inst_lock: 
        save_ros_msg_as_json(inst_msg, os.path.join(run_save_path, "INST_POSE"+str(img_save_counter)+".json"))

    print("Saved capture ", img_save_counter)
    img_save_counter += 1

# def save_all_ros_msg():
#     global img_save_counter

#     with rimg_lock:
#         save_ros_msg_as_img(rimg_msg, os.path.join(run_save_path, "rimg"+str(img_save_counter)+".png"))

#     with limg_lock: 
#         save_ros_msg_as_img(limg_msg, os.path.join(run_save_path, "limg"+str(img_save_counter)+".png"))

#     print("Saved capture ", img_save_counter)
#     img_save_counter += 1

##############################################################################


def create_unique_output_folder():
    if not os.path.isdir(OUTPUT_PATH):
        os.makedirs(OUTPUT_PATH)

    counter = 1
    subfolder_path = os.path.join(OUTPUT_PATH,"run_"+str(counter))
    while os.path.isdir(subfolder_path):
        subfolder_path = os.path.join(OUTPUT_PATH,"run_"+str(counter))
        counter += 1

    os.makedirs(subfolder_path)


    return subfolder_path


def handle_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global img_save_counter
        img_save_counter = max( img_save_counter - 1, 0)
        print("Next capture will overwrite capture", img_save_counter)

    if event == cv2.EVENT_RBUTTONDOWN:
        save_all_ros_msg()


if __name__ == u'__main__':
    rospy.init_node(u'atracsys_listener', anonymous=True)

    # ROS SUBSCRIBERS
    sub1 = threading.Thread(target=hmd_listener)
    sub1.daemon = True
    sub1.start()

    sub2 = threading.Thread(target=inst_listener)
    sub2.daemon = True
    sub2.start()

    # sub1 = threading.Thread(target=limg_listener)
    # sub1.daemon = True
    # sub1.start()

    # sub2 = threading.Thread(target=rimg_listener)
    # sub2.daemon = True
    # sub2.start()

    run_save_path = create_unique_output_folder()

    cv2.namedWindow(u'dummy window', cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL )
    cv2.setMouseCallback(u'dummy window', handle_mouse)

    dummy_image = cv2.imread(u'dummy_image.jpeg')

    cv2.imshow(u'dummy window', dummy_image)

    while True:
        key = cv2.waitKey(0) & 0xFF
        
        if key == ord(u'q'):
            break
        elif key == ord(u' '):
            save_all_ros_msg()
        elif key == ord(u','):
            img_save_counter = max( img_save_counter - 1, 0)
            print("Next capture will overwrite capture", img_save_counter)
        elif key == ord(u'.'):
            img_save_counter += 1
            print("Next capture will overwrite capture", img_save_counter)