import os
import re
import cv2
import shutil
import numpy as np
# from Solver import solver
# from dataLoader import dataLoader
from natsort import natsorted
from cv_bridge import CvBridge
import argparse
# import sys
# import pandas as pd

# from util.bag2hdf5 import main

br = CvBridge()

def rectifyStereo(rectify_dir, intrinsics_dir, leftOnly):

    left_dir = rectify_dir + '/limg'
    right_dir = rectify_dir + '/rimg'

    # left_out_dir = rectify_dir + '/rectified_left'
    # right_out_dir = rectify_dir + '/rectified_right'
    # if not os.path.exists(left_out_dir):
    #     os.makedirs(left_out_dir)
    #     os.makedirs(right_out_dir)
    # if os.listdir(left_out_dir):
    #     shutil.rmtree(left_out_dir)
    #     shutil.rmtree(right_out_dir)
    #     os.makedirs(left_out_dir)
    #     os.makedirs(right_out_dir)
    
    # d_l = np.load(f'{intrinsics_dir}/d_l.npy')
    d_l = d_r = np.zeros([1, 5])
    # d_r = np.load(f'{intrinsics_dir}/d_r.npy')
    # d_r = d_r[:5]
    M_l = np.load(f'{intrinsics_dir}/M_l.npy')
    M_r = np.load(f'{intrinsics_dir}/M_r.npy')
    R = np.load(f'{intrinsics_dir}/R.npy')
    T = np.load(f'{intrinsics_dir}/T.npy')
    
    # cv record video
    fps = 30
    size = (1920, 1080)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    saveName = os.path.join(rectify_dir, 'rectified_left.avi')
    videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)


    imgs_left = natsorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if ".jpeg" in f])
    imgs_right = natsorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if ".jpeg" in f])

    imsize = (1920, 1080)
    R_l, R_r, P_l, P_r, Q = cv2.stereoRectify(M_l, d_l, M_r, d_r, imsize, R, T, flags=cv2.CALIB_ZERO_DISPARITY)[0:5]
    # np.save('hand_eye_calib_rectified/P_r', P_r)
    mapLx, mapLy = cv2.initUndistortRectifyMap(M_l,
                                               d_l,
                                               R_l, P_l, imsize, cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(M_r,
                                               d_r,
                                               R_r, P_r, imsize, cv2.CV_32FC1)

    for f_left, f_right in zip(imgs_left, imgs_right):
        img_left = cv2.imread(f_left)
        img_right = cv2.imread(f_right)

        # get image index
        idx = re.findall('\d+', str(f_left))[-1]

        recImgL = cv2.remap(img_left, mapLx, mapLy, cv2.INTER_LINEAR)
        recImgR = cv2.remap(img_right, mapRx, mapRy, cv2.INTER_LINEAR)

        if leftOnly:
            # cv2.imwrite(rectify_dir + f'/rectified_left/{idx}.jpeg', recImgL)
            # print(f'Finish writing {idx}!')
            # cv2.imshow('limg', recImgL)
            videoWriter.write(recImgL)
            print(f'finish rectification frame {idx}.')
        else:
            # cv2.imwrite(rectify_dir + f'/rectified_right/{idx}.jpeg', recImgR)
            # cv2.imwrite(rectify_dir + f'/rectified_left/{idx}.jpeg', recImgL)
            # print(f'Finish writing {idx}!')
            cv2.imshow('limg', recImgL)
            cv2.imshow('rimg', recImgR)

    videoWriter.release()

def main():
    parser = argparse.ArgumentParser(description="Make rectified images a video.")
    parser.add_argument("-r", "--rectify_dir", help="dir to store rectified video", default="../../Desktop/data/", type=str)
    parser.add_argument("-i", "--intrinsics_dir", help="dir to store camera intrinsic params", default="../params", type=str)
    parser.add_argument("-l", "--leftOnly", help="only output left video or both sides", default=True, type=bool)
    args = parser.parse_args()
    
    # rectify_dir = "../../Desktop/data/623_2"
    # intrinsics_dir = "../params"
    rectifyStereo(args.rectify_dir, args.intrinsics_dir, args.leftOnly)

if __name__ == '__main__':
    main()