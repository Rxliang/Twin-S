import numpy as np
import cv2
import os
import re
import shutil
import matplotlib.pyplot as plt
from natsort import natsorted
import seaborn as sns
from cv2 import aruco

def findRectifiedCorners(rectify_dir, checkerboard=True):

    left_out_dir = rectify_dir + '/rectified_left'
    right_out_dir = rectify_dir + '/rectified_right'

    imgs_left = natsorted([os.path.join(left_out_dir, f) for f in os.listdir(left_out_dir) if ".jpeg" in f])
    imgs_right = natsorted([os.path.join(right_out_dir, f) for f in os.listdir(right_out_dir) if ".jpeg" in f])

    if checkerboard:
        CHECKERBOARD = (8 - 1, 11 - 1)
        # Defining the world coordinates for 3D points
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * 2.0  # 2mm board
        # Creating vector to store vectors of 3D points for each checkerboard image
        objpoints = []
        # Creating vector to store vectors of 2D points for each checkerboard image
        imgpoints_left = []
        imgpoints_right = []
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
        count = 0
        for f_left, f_right in zip(imgs_left, imgs_right):
            img_left = cv2.imread(f_left, cv2.IMREAD_GRAYSCALE)
            img_right = cv2.imread(f_right, cv2.IMREAD_GRAYSCALE)

            # Find the chess board corners
            # If desired number of corners are found in the image then ret = true
            ret_left, corners_left = cv2.findChessboardCorners(img_left, CHECKERBOARD,
                                                               cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            ret_right, corners_right = cv2.findChessboardCorners(img_right, CHECKERBOARD,
                                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

            """
            If desired number of corner are detected,
            we refine the pixel coordinates and display
            them on the images of checker board
            """
            if ret_left and ret_right:
                print(f'Rectified chess corners found {count}')

                objpoints.append(objp)
                # refining pixel coordinates for given 2d points.
                corners2_left = cv2.cornerSubPix(img_left, corners_left, (11, 11), (-1, -1), criteria)
                imgpoints_left.append(corners2_left)
                # Draw and display the corners
                img_left = cv2.drawChessboardCorners(img_left, CHECKERBOARD, corners2_left, ret_left)

                # refining pixel coordinates for given 2d points.
                corners2_right = cv2.cornerSubPix(img_right, corners_right, (11, 11), (-1, -1), criteria)
                imgpoints_right.append(corners2_right)
                # Draw and display the corners
                img_right = cv2.drawChessboardCorners(img_right, CHECKERBOARD, corners2_right, ret_right)

                img_left = cv2.resize(img_left, [960, 540])
                img_right = cv2.resize(img_right, [960, 540])
                img = np.hstack((img_left, img_right))
                cv2.imshow('rec', img)
                cv2.waitKey(0)
            else:
                print('no corners found in file', f_left)
                os.remove(f_left)
                os.remove(f_right)

            first_left = imgpoints_left[count][0,:,:]
            first_right = imgpoints_right[count][0,:,:]
            if abs(first_left[0,1] - first_right[0,1]) > 1:
                print(f'>>>>>REVERSE {count}>>>>>')
                imgpoints_right[count] = imgpoints_right[count][::-1]
            count += 1
            np.save(rectify_dir + '/rectified_imgpoints_left', imgpoints_left)
            np.save(rectify_dir + '/rectified_imgpoints_right', imgpoints_right)
    else:
        # ChAruco board variables
        CHARUCOBOARD_ROWCOUNT = 8
        CHARUCOBOARD_COLCOUNT = 11
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)

        # Create constants to be passed into OpenCV and Aruco methods
        CHARUCO_BOARD = aruco.CharucoBoard_create(
            squaresX=CHARUCOBOARD_COLCOUNT,
            squaresY=CHARUCOBOARD_ROWCOUNT,
            squareLength=0.03,
            markerLength=0.02,
            dictionary=ARUCO_DICT)

        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        objpoints = []
        imgpoints_left = []
        imgpoints_right = []

        objp = np.zeros((1, (CHARUCOBOARD_ROWCOUNT - 1) * (CHARUCOBOARD_COLCOUNT - 1), 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHARUCOBOARD_ROWCOUNT - 1, 0:CHARUCOBOARD_COLCOUNT - 1].T.reshape(-1, 2) * 3.0 # 3mm board
        # use Charuco Target
        for f_left, f_right in zip(imgs_left, imgs_right):
            img_left = cv2.imread(f_left)
            img_right = cv2.imread(f_right)
            # Grayscale the image
            gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

            param = aruco.DetectorParameters_create()
            param.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            # Find aruco markers in the query image
            corners_left, ids_left, _ = aruco.detectMarkers(gray_left, ARUCO_DICT, parameters=param)
            corners_right, ids_right, _ = aruco.detectMarkers(gray_right, ARUCO_DICT, parameters=param)

            # Outline the aruco markers found in our query image
            img_l = aruco.drawDetectedMarkers(
                image=img_left,
                corners=corners_left)
            img_r = aruco.drawDetectedMarkers(
                image=img_right,
                corners=corners_right)

            # Get charuco corners and ids from detected aruco markers
            response_l, charuco_corners_left, charuco_ids_left = aruco.interpolateCornersCharuco(
                markerCorners=corners_left,
                markerIds=ids_left,
                image=gray_left,
                board=CHARUCO_BOARD)
            response_r, charuco_corners_right, charuco_ids_right = aruco.interpolateCornersCharuco(
                markerCorners=corners_right,
                markerIds=ids_right,
                image=gray_right,
                board=CHARUCO_BOARD)

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
            # refining pixel coordinates for given 2d points.
            charuco_corners_left = cv2.cornerSubPix(gray_left, charuco_corners_left, (11, 11), (-1, -1), criteria)
            charuco_corners_right = cv2.cornerSubPix(gray_right, charuco_corners_right, (11, 11), (-1, -1), criteria)

            # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 20 squares
            if response_l == response_r and response_l == (CHARUCOBOARD_ROWCOUNT - 1) * (CHARUCOBOARD_COLCOUNT - 1):
                print('charuco corners found')
                objpoints.append(objp)
                # Add these corners and ids to our calibration arrays
                imgpoints_left.append(charuco_corners_left)
                imgpoints_right.append(charuco_corners_right)
            else:
                print('no corners found in file', f_left)
                # os.remove(f_left)
                # os.remove(f_right)
        np.save(rectify_dir + '/rectified_imgpoints_left', imgpoints_left)
        np.save(rectify_dir + '/rectified_imgpoints_right', imgpoints_right)

def rectifyStereo(rectify_dir, intrinsics_dir, leftOnly):

    left_dir = rectify_dir + '/prepare_left'
    right_dir = rectify_dir + '/prepare_right'

    left_out_dir = rectify_dir + '/rectified_left'
    right_out_dir = rectify_dir + '/rectified_right'

    if not os.path.exists(left_out_dir):
        os.makedirs(left_out_dir)
        os.makedirs(right_out_dir)
    if os.listdir(left_out_dir):
        shutil.rmtree(left_out_dir)
        shutil.rmtree(right_out_dir)
        os.makedirs(left_out_dir)
        os.makedirs(right_out_dir)

    # d_l = np.load(f'{intrinsics_dir}/d_l.npy')
    d_l = d_r = np.zeros([1, 5])
    # d_r = np.load(f'{intrinsics_dir}/d_r.npy')
    # d_r = d_r[:5]
    M_l = np.load(f'{intrinsics_dir}/M_l.npy')
    M_r = np.load(f'{intrinsics_dir}/M_r.npy')
    R = np.load(f'{intrinsics_dir}/R.npy')
    T = np.load(f'{intrinsics_dir}/T.npy')

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

    np.save(f'{rectify_dir}/Q', Q)
    np.save(f'{rectify_dir}/P_r.npy', P_r)

    for f_left, f_right in zip(imgs_left, imgs_right):
        img_left = cv2.imread(f_left)
        img_right = cv2.imread(f_right)

        # get image index
        idx = re.findall('\d+', str(f_left))[-1]

        recImgL = cv2.remap(img_left, mapLx, mapLy, cv2.INTER_LINEAR)
        recImgR = cv2.remap(img_right, mapRx, mapRy, cv2.INTER_LINEAR)

        if leftOnly:
            cv2.imwrite(rectify_dir + f'/rectified_left/{idx}.jpeg', recImgL)
            print(f'Finish writing {idx}!')
        else:
            cv2.imwrite(rectify_dir + f'/rectified_right/{idx}.jpeg', recImgR)
            cv2.imwrite(rectify_dir + f'/rectified_left/{idx}.jpeg', recImgL)
            print(f'Finish writing {idx}!')


if __name__ == '__main__':
    rectify_dir = 'Charuco_Calib_419'
    intrinsic_dir = 'intrinsic_params_421'
    rectifyStereo(rectify_dir, intrinsic_dir, leftOnly=False)

    # findRectifiedCorners(rectify_dir, checkerboard=True)
    # lpoints = np.load(rectify_dir + '/rectified_imgpoints_left.npy')
    # rpoints = np.load(rectify_dir + '/rectified_imgpoints_right.npy')
    #
    # lpoints = lpoints.reshape([-1, 2])
    # rpoints = rpoints.reshape([-1, 2])
    #
    # disp = rpoints - lpoints
    # print(abs(disp > 1).sum())
    # sns.distplot(disp[:, 1], bins=50, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"})
    # plt.show()
    # print('Mean:', np.mean(disp[:,1]), 'std:', disp[:, 1].std())
