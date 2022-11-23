from glob import glob
import rosbag
import rospy
import ros_numpy
import bagpy
import cv2
import numpy as np
import os
import time
import matplotlib.pyplot as plt
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
from depthmap import SqueezedNorm
from depthmap import rospc_to_o3dpc
import open3d
from ros_tools import rostools


def initialization():
    global file , args, path
    
    parser = argparse.ArgumentParser(description="Extract images from rosbag.")
    parser.add_argument('--bag',dest="bag_file", help="Input ROS bag directory", default='./test.bag', type=str)
    parser.add_argument('--outdir',dest="output_dir", help="Output directory.", default='./Data', type=str)
    parser.add_argument('--stereo',dest="stereo", help="Stereo or monocular.", action='store_true')
    parser.add_argument('--limg',dest="limg", help="Left image only.", action='store_true')
    parser.add_argument('--panel',dest="pan", help="With phacon panel or not.", action='store_false')
    parser.add_argument('--l_topic',dest="l_img_topic", help="Topic of left image.", default='/sync_limage/compressed', type=str)
    parser.add_argument('--r_topic',dest="r_img_topic", help="Topic of right image.", default='/fwd_rimage/compressed', type=str)
    parser.add_argument('--segm',dest="segm", help="Segmentation masks.", action='store_true')
    parser.add_argument('--depth',dest="depth", help="Depth map.", action='store_true')
    parser.add_argument('--segm_topic',dest="segm_topic", help="Topic of segmentation.", default='/sync_segm/compressed', type=str)
    parser.add_argument('--depth_topic',dest="depth_topic", help="Topic of depth.", default='/sync_depthData', type=str)
    parser.add_argument('--sim_topic',dest="sim_topic", help="Topic of stereoL images in ambf.", default='/sync_sim/compressed', type=str)
    parser.add_argument('--pose',dest="pose", help="Poses of optical tracking.", action='store_true')
    parser.add_argument('--sim',dest="sim", help="StereoL images of AMBF.", action='store_true')

    # segm_topic = '/fwd_segm/compressed'
    # depth_topic = '/fwd_depthData'
    stereoL_topic = '/ambf/env/cameras/stereoL/ImageData/compressed'
    args = parser.parse_args()
    args.output_dir = args.bag_file[:-4]

    valid = rt.verify_cv_bridge()
    # verify_ROS_connection()
    limg_path = args.output_dir +'/limg/'
    rimg_path = args.output_dir +'/rimg/'
    segm_path = args.output_dir +'/segm_mask/'
    depth_path = args.output_dir +'/depth_map/'
    sim_path = args.output_dir +'/sim_img/'

    path = [limg_path, rimg_path, segm_path, depth_path, sim_path]

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
        elif args.limg:
            if not os.path.exists(limg_path):
                os.makedirs(limg_path)
                print("Create Folder at Designated Address limg...")
        if args.segm:
            if not os.path.exists(segm_path):
                os.makedirs(segm_path)
                print("Create Folder at Designated Address segm_mask...")
        if args.depth:
            if not os.path.exists(depth_path):
                os.makedirs(depth_path)
                print("Create Folder at Designated Address depth_path...")
        if args.sim:
            if not os.path.exists(sim_path):
                os.makedirs(sim_path)
                print("Create Folder at Designated Address sim_path...")
    else:
        print("Failed")


def bag2csv(args):
    b = bagpy.bagreader(args.bag_file)
    if args.pan:
        b.message_by_topic('/fwd_pose_pan')
        b.message_by_topic('/fwd_pose_drill')
        b.message_by_topic('/fwd_pose_camhand')
    else:
        b.message_by_topic('/fwd_pose_drill')
        b.message_by_topic('/fwd_pose_camhand')
    print("Complete Saving csv.")


def bag2images(args):
    global path
    bridge = CvBridge()
    scale = 0.180
    start = time.time()
    img_sec_list_l, img_sec_list= [], []
    

    with rosbag.Bag(args.bag_file, 'r') as bag:
        if args.stereo:
            start = time.time()
            rt.saveImagesFromBag(bag, args.l_img_topic, img_sec_list_l, path[0])

            rt.saveImagesFromBag(bag, args.r_img_topic, img_sec_list, path[1])

        elif args.limg:
            rt.saveImagesFromBag(bag, args.l_img_topic, img_sec_list, path[0])

        if args.segm:
            rt.saveImagesFromBag(bag, args.segm_topic, img_sec_list, path[2])

        if args.depth:
            rt.saveDepthImagesFromBag(bag, args.depth_topic, scale, path[3])
        
        if args.sim:
            rt.saveImagesFromBag(bag, args.sim_topic, img_sec_list, path[4])
        end = time.time()
        print(end - start)
        print("Complete Saving")
        return img_sec_list

def main():
    global args
    initialization()
    if args.pose:
        bag2csv(args)
    img_sec_list = bag2images(args)




if __name__ == '__main__':
    rt = rostools()
    bridge = CvBridge()
    main()
    # print(len(time_stamp))
