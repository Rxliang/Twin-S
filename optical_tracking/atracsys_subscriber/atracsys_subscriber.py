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

base_lock = threading.Lock()
tool_lock = threading.Lock()
limg_lock = threading.Lock()
rimg_lock = threading.Lock()

base_msg = u""
tool_msg = u""
limg_msg = CompressedImage()
rimg_msg = CompressedImage()

###############################################################################

# create a callback and listener (and lock) for each ros topic

def base_callback(msg):
    global run_save_path_list, img_save_counter, base_msg, base_lock
    # print(u'msg',msg)
    with base_lock:
        # print("In Lock")
        base_msg = msg

def base_listener():
    rospy.Subscriber(u"/atracsys/Panel/measured_cp", PoseStamped, base_callback)
    rospy.spin()

def tool_callback(msg):
    global run_save_path_list, img_save_counter, tool_msg, tool_lock
    # print(u'msg',msg)
    with tool_lock:
        tool_msg = msg

def tool_listener():
    rospy.Subscriber(u"/atracsys/Pointer/measured_cp", PoseStamped, tool_callback)
    rospy.spin()

#callback and listener for img topics
# def limg_callback(msg):
#     global run_save_path_list, img_save_counter, limg_msg, limg_lock
#     # print(u'msg',msg)
#     with limg_lock:
#         limg_msg = msg

# def limg_listener():
#     rospy.Subscriber(u"/zedm/zed_node/left/image_rect_color/compressed", CompressedImage, limg_callback)
#     rospy.spin()

# def rimg_callback(msg):
#     global run_save_path_list, img_save_counter, rimg_msg, rimg_lock
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

    with base_lock:
        save_ros_msg_as_json(base_msg, os.path.join(run_save_path_list[0], "BASE_POSE"+str(img_save_counter)+".json"))

    with tool_lock: 
        save_ros_msg_as_json(tool_msg, os.path.join(run_save_path_list[1], "TOOL_POSE"+str(img_save_counter)+".json"))

    print("Saved capture ", img_save_counter)
    img_save_counter += 1

# def save_all_ros_msg():
#     global img_save_counter

#     with rimg_lock:
#         save_ros_msg_as_img(rimg_msg, os.path.join(run_save_path_list, "rimg"+str(img_save_counter)+".png"))

#     with limg_lock: 
#         save_ros_msg_as_img(limg_msg, os.path.join(run_save_path_list, "limg"+str(img_save_counter)+".png"))

#     print("Saved capture ", img_save_counter)
#     img_save_counter += 1

##############################################################################


def create_unique_output_folder():
    if not os.path.isdir(OUTPUT_PATH):
        os.makedirs(OUTPUT_PATH)

    counter = 1
    subfolder_path = os.path.join(OUTPUT_PATH,"phantom_"+str(counter))
    while os.path.isdir(subfolder_path):
        subfolder_path = os.path.join(OUTPUT_PATH,"phantom_"+str(counter))
        counter += 1

    os.makedirs(subfolder_path)
    subsub_path_list = [os.path.join(subfolder_path,"base"), os.path.join(subfolder_path,"tool")]
    if not os.path.isdir(subsub_path_list[0]):
        os.makedirs(subsub_path_list[0])
        os.makedirs(subsub_path_list[1])

    return subsub_path_list


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
    sub1 = threading.Thread(target=base_listener)
    sub1.daemon = True
    sub1.start()

    sub2 = threading.Thread(target=tool_listener)
    sub2.daemon = True
    sub2.start()

    # sub1 = threading.Thread(target=limg_listener)
    # sub1.daemon = True
    # sub1.start()

    # sub2 = threading.Thread(target=rimg_listener)
    # sub2.daemon = True
    # sub2.start()

    run_save_path_list = create_unique_output_folder()

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