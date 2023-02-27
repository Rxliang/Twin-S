import sys
import time
import rosbag
import open3d as o3d
import argparse
from glob import glob
sys.path.append('../')
sys.path.insert(0, '/home/shc/RoboMaster/util')
import numpy as np
from dataLoader import dataLoader
from Solver import solver
from ros_tools import rostools
import os
from natsort import natsorted
import cv2
import pickle
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError



def init():

    global file , args, path
    parser = argparse.ArgumentParser(description="Extract images from rosbag.")

    parser.add_argument('--bag',dest="bag_file", help="Input ROS bag directory", default='./test.bag', type=str)
    parser.add_argument('--ambf_pcd_topic',dest="ambf_pcd_topic", help="Topic of pointcloud2 from AMBF.", default='/fwd_sim_pointcloud', type=str)
    parser.add_argument('--zed_pcd_topic',dest="zed_pcd_topic", help="Topic of pointcloud2 from ZED.", default='/fwd_pointcloud', type=str)
    args = parser.parse_args()

    output_dir = args.bag_file[:-4]
    ambf_pcd_path = output_dir +'/ambf_pointcloud/'
    zed_pcd_path = output_dir +'/zed_pointcloud/'

    path = [ambf_pcd_path, zed_pcd_path]

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print("Create Folder at Designated Address...")
    if not os.path.exists(ambf_pcd_path):
        os.makedirs(ambf_pcd_path)
        print("Create Folder at Designated Address ambf_pcd_path...")
    if not os.path.exists(zed_pcd_path):
        os.makedirs(zed_pcd_path)
        print("Create Folder at Designated Address zed_pcd_path...")


def bag2Pointcloud():
    global file , args, path

    init()

    with rosbag.Bag(args.bag_file, 'r') as bag:
        count = 0
        start = time.time()
        for topic, depth_msg, t in bag.read_messages(args.ambf_pcd_topic):
            ambf_pcd = rt.convertCloudFromRosToOpen3d(depth_msg)
            
            print(path[0] +' '+ str(count))
            ambf_file_name = os.path.join(path[0], str(count)) + '.pcd'
            o3d.io.write_point_cloud(ambf_file_name, ambf_pcd)
            count += 1
            if count == 1:
                break
        

        count = 0
        for topic, depth_msg, t in bag.read_messages(args.zed_pcd_topic):
            zed_pcd = rt.convertCloudFromRosToOpen3d(depth_msg)
            
            print(path[1] +' '+ str(count))
            zed_file_name = os.path.join(path[1], str(count)) + '.pcd'
            o3d.io.write_point_cloud(zed_file_name, zed_pcd)
            count += 1

            o3d.visualization.draw_geometries([zed_pcd])
            
        end = time.time()
        print(end - start)
        print("Complete Saving")


if __name__ == '__main__':
    ld = dataLoader()
    sol = solver()
    rt = rostools()
    bridge = CvBridge()
    bag2Pointcloud()
    