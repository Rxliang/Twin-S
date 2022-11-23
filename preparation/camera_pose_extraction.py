import stereo_vision_pipeline as sp
from glob import glob
import rosbag
import rospy
import cv2
import numpy as np
import os
import time
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
bridge = CvBridge()


def verify_cv_bridge():
    arr = np.zeros([480, 640])
    msg = bridge.cv2_to_imgmsg(arr)
    try:
        bridge.imgmsg_to_cv2(msg)
    except ImportError:
        # log.log(logging.WARNING, "libcv_bridge.so: cannot open shared object file. Please source ros env first.")
        return False

    return True

def initialization():
    global file , args
    
    parser = argparse.ArgumentParser(description="Extract images from rosbag.")
    # parser.add_argument("img_dir", help="Input image directory", default='./test.bag', type=str)
    parser.add_argument('--bag',dest="bag_file", help="Input ROS bag directory", default='./test.bag', type=str)
    parser.add_argument('--outdir',dest="output_dir", help="Output directory.", default='./Data', type=str)
    parser.add_argument('--stereo',dest="stereo", help="Stereo or monocular.", type=bool)
    parser.add_argument('--topic',dest="img_topic", help="Topic of image.", default='/camera/color/image_raw/compressed', type=str)
    parser.add_argument('--l_topic',dest="l_img_topic", help="Topic of left image.", default='/fwd_limage/compressed', type=str)
    parser.add_argument('--r_topic',dest="r_img_topic", help="Topic of right image.", default='/fwd_rimage/compressed', type=str)
    
    args = parser.parse_args()
    valid = verify_cv_bridge()
    # verify_ROS_connection()
    limg_path = args.output_dir +'/limg/'
    rimg_path = args.output_dir +'/rimg/'
    img_path = args.output_dir +'/img/'
    if valid:
        if not os.path.exists(args.output_dir):
                os.makedirs(args.output_dir)
                print("Create Folder at Designated Address...")
        if args.stereo:
            if not os.path.exists(limg_path):
                os.makedirs(limg_path)
                print("Create Folder at Designated Address limg...")
            if not os.path.exists(rimg_path):
                os.makedirs(rimg_path)
                print("Create Folder at Designated Address rimg...")
            time_str = time.strftime("%Y%m%d_%H%M%S")
            # file = h5py.File(args.output_dir + '/' + time_str + ".hdf5", "w")
        else:
            if not os.path.exists(img_path):
                os.makedirs(img_path)
                print("Create Folder at Designated Address img...")
        #Under Construction
    else:
        print("Failed")

def bag2images(args):
    bridge = CvBridge()
    count = 0
    start = time.time()
    img_sec_list_l, img_sec_list= [], []
    with rosbag.Bag(args.bag_file, 'r') as bag:
        if args.stereo:
            start = time.time()
            for topic, msg, t in bag.read_messages(topics = args.l_img_topic):
                img_sec = msg.header.stamp.nsecs
                img_sec_list_l.append(img_sec)

                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except CvBridgeError as e:
                    print(e)
                image_path = args.output_dir +'/limg/' + str(count) + '.jpeg'
                pil_image = Image.fromarray(cv_image[:,:,::-1])
                pil_image.save(image_path)
                count += 1
                print(count)

            count = 0
            for topic, msg, t in bag.read_messages(topics = args.r_img_topic):
                img_sec = msg.header.stamp.nsecs
                img_sec_list_l.append(img_sec)

                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except CvBridgeError as e:
                    print(e)
                image_path = args.output_dir +'/rimg/' + str(count) + '.jpeg'
                pil_image = Image.fromarray(cv_image[:,:,::-1])
                pil_image.save(image_path)
                count += 1
                print(count)

            end = time.time()
            print(end - start)
            print("Complete Saving")
            return img_sec_list_l

        else:
            for topic, msg, t in bag.read_messages(topics = args.img_topic):
                img_sec = msg.header.stamp.nsecs
                img_sec_list.append(img_sec)

                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except CvBridgeError as e:
                    print(e)
                image_path = args.output_dir +'/img/' + str(count) + '.jpeg'
                pil_image = Image.fromarray(cv_image[:,:,::-1])
                pil_image.save(image_path)
                count += 1
                print(count)

            end = time.time()
            print(end - start)
            print("Complete Saving")
            return img_sec_list

def main():
    global args
    initialization()
    # img_sec_list = bag2images(args)
    # np.save('time_stamp',img_sec_list)

    data_dir = args.output_dir +'/img/'
    cam_mtx = np.zeros((4,4))
    cam_dist = np.zeros([5])

    marker_poses, imgpts_list = sp.getChessPoses(data_dir, cam_mtx, cam_dist, charuco=False, pattern=(6,8), width=20)




if __name__ == '__main__':
    main()
    time_stamp = np.load('time_stamp.npy')
    # print(len(time_stamp))
