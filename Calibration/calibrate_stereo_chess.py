import os
import sys
import shutil
import cv2
import matplotlib.pyplot as plt
import numpy as np
from natsort import natsorted
import seaborn as sns
from cv2 import aruco

def stereoCalibration(inputImgs_dir, rm_num=0, test_mode=False):
    # downsample_ratio = 0.5

    outParams_dir = inputImgs_dir
    # Defining the dimensions of checkerboard
    CHECKERBOARD = (8 - 1, 11 - 1)  # subtract 1
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints_left = []
    imgpoints_right = []

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * 2.0  # 2mm board
    prev_img_shape = None

    # read image
    left_dir = inputImgs_dir + "/calib_left"
    right_dir = inputImgs_dir + "/calib_right"

    file_list = os.listdir(left_dir)
    # make a calib_right containing images of index with calib_left
    if not os.path.exists(right_dir):
        os.makedirs(right_dir)
    # if os.listdir(right_dir):
    #     shutil.rmtree(right_dir)
    #     os.makedirs(right_dir)

    for file in file_list:
        ori_dir = inputImgs_dir + "/rimg"
        move_item = os.path.join(ori_dir, file)
        shutil.copy(move_item, right_dir)

    imgs_left = natsorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if ".jpeg" in f])
    imgs_right = natsorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if ".jpeg" in f])

    # test mode used to test calibration sensitivity by feeding different number of images
    if test_mode:
        test_left_dir = inputImgs_dir + "/test_sensitivity_left"
        test_right_dir = inputImgs_dir + "/test_sensitivity_right"
        if not os.path.exists(test_left_dir) or not os.path.exists(test_right_dir):
            os.makedirs(test_left_dir)
            os.makedirs(test_right_dir)
        for file in file_list:
            shutil.copy(os.path.join(left_dir,file), test_left_dir)
            shutil.copy(os.path.join(right_dir,file), test_right_dir)
        test_file_list = os.listdir(left_dir)
        for i in range(rm_num):
            rm_left = os.path.join(test_left_dir,  test_file_list[0])
            rm_right = os.path.join(test_right_dir, test_file_list[0])
            os.remove(rm_left)
            os.remove(rm_right)
            test_file_list.pop(0)

        imgs_left = natsorted([os.path.join(test_left_dir, f) for f in os.listdir(test_left_dir) if ".jpeg" in f])
        imgs_right = natsorted([os.path.join(test_right_dir, f) for f in os.listdir(test_right_dir) if ".jpeg" in f])

    allCorners = []
    allIds = []
    for f_left, f_right in zip(imgs_left, imgs_right):
        img_left = cv2.imread(f_left, cv2.IMREAD_GRAYSCALE)
        img_right = cv2.imread(f_right, cv2.IMREAD_GRAYSCALE)
        # # Important!!: downsample images
        # img_left = cv2.resize(img_left, None, fx=downsample_ratio, fy=downsample_ratio, interpolation=cv2.INTER_AREA)
        # img_right = cv2.resize(img_right, None, fx=downsample_ratio, fy=downsample_ratio, interpolation=cv2.INTER_AREA)

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
            print('chess corners found')
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2_left = cv2.cornerSubPix(img_left, corners_left, (11, 11), (-1, -1), criteria)
            imgpoints_left.append(corners2_left)
            # Draw and display the corners
            img_left = cv2.drawChessboardCorners(img_left, CHECKERBOARD, corners2_left, ret_left)
            # cv2.imshow('img', img_left)
            # cv2.waitKey(500)

            # refining pixel coordinates for given 2d points.
            corners2_right = cv2.cornerSubPix(img_right, corners_right, (11, 11), (-1, -1), criteria)
            imgpoints_right.append(corners2_right)
            # Draw and display the corners
            img_right = cv2.drawChessboardCorners(img_right, CHECKERBOARD, corners2_right, ret_right)
        else:
            print('no corners found in file', f_left)
            os.remove(f_left)
            os.remove(f_right)

        # plt.imshow(img, cmap='gray')
        # plt.show()

    imsize = img_left.shape[::-1]
    print('imgsize:', imsize)

    # try Calibration
    try:
        [ret_left, M_left, d_left, r_left, t_left] = cv2.calibrateCamera(objpoints, imgpoints_left, imsize, None,
                                                                         None, flags=cv2.CALIB_FIX_PRINCIPAL_POINT +
                                                                                     cv2.CALIB_ZERO_TANGENT_DIST +
                                                                                     cv2.CALIB_FIX_K1 + cv2.CALIB_FIX_K2 +
                                                                                     cv2.CALIB_FIX_K3)

        print("Left rep Error:", ret_left)
        [ret_right, M_right, d_right, r_right, t_right] = cv2.calibrateCamera(objpoints, imgpoints_right, imsize, None,
                                                                              None, flags=cv2.CALIB_FIX_PRINCIPAL_POINT +
                                                                                          cv2.CALIB_ZERO_TANGENT_DIST +
                                                                                          cv2.CALIB_FIX_K1 + cv2.CALIB_FIX_K2 +
                                                                                          cv2.CALIB_FIX_K3)

        print("Right rep Error:", ret_right)
        # np.savez(args.file,ret=ret,mtx=cameraMatrix,dist=disCoeffs,rvecs=rvecs,tvecs=tvecs)
        print('M_left:\n', M_left)
        print('d_left:\n', d_left)
        print('M_right:\n', M_right)
        print('d_right:\n', d_right)

        # 反向投影误差
        mean_error_l = 0
        mean_error_r = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], r_left[i], t_left[i], M_left, d_left)
            imgpoints3, _ = cv2.projectPoints(objpoints[i], r_right[i], t_right[i], M_right, d_right)
            error_l = cv2.norm(imgpoints_left[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            error_r = cv2.norm(imgpoints_right[i], imgpoints3, cv2.NORM_L2) / len(imgpoints3)
            mean_error_l += error_l
            mean_error_r += error_r
        print("l_reproj error: ", mean_error_l / len(objpoints))
        print("r_reproj error: ", mean_error_r / len(objpoints))

        # dist = 0
        # # recover pose by PnP
        # for f_left in imgs:
        #     img_left = cv2.imread(f_left, cv2.IMREAD_GRAYSCALE)
        #     ret_left, corners_left = cv2.findChessboardCorners(img_left, CHECKERBOARD,
        #                                                        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK +
        #                                                        cv2.CALIB_CB_NORMALIZE_IMAGE)
        #     if ret_left:
        #         corners2_left = cv2.cornerSubPix(img_left, corners_left, (11, 11), (-1, -1), criteria)
        #         # Find the rotation and translation vectors.
        #         ret_left, r_left, t_left = cv2.solvePnP(objp, corners2_left, M_left, d_left)
        #         dist = dist + t_left
        #         print(ret_left, r_left, t_left)
        # print("average distance", dist / len(imgs))


        # stereo flags
        flags = 0
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        flags |= cv2.CALIB_FIX_K1
        flags |= cv2.CALIB_FIX_K2
        flags |= cv2.CALIB_FIX_K3
        flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M_l, d_l, M_r, d_r, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, M_left,
                                                                  d_left, M_right, d_right, imsize,
                                                                  criteria=stereocalib_criteria, flags=flags)


        print("stereo param:")
        print('ret:\n', ret)
        print('M_l:\n', M_l)
        print('d_l:\n', d_l)
        print('M_r:\n', M_r)
        print('d_r:\n', d_r)
        print('R:\n', R)
        print('T:\n', T)

        if test_mode:
            return M_l, M_r, T
        else:
            save_dir = outParams_dir
            np.save(save_dir + '/M_l', M_l)
            np.save(save_dir + '/d_l', d_l)
            np.save(save_dir + '/M_r', M_r)
            np.save(save_dir + '/d_r', d_r)
            np.save(save_dir + '/R', R)
            np.save(save_dir + '/T', T)

    except ValueError as e:
        print(e)
    except NameError as e:
        print(e)
    except AttributeError as e:
        print(e)
    except:
        print("calibrateCameraCharuco fail:", sys.exc_info()[0])

    # undistort image to verify if radial distortion makes sense at all
    # imgs_left = natsorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if ".png" in f])
    # imgs_right = natsorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if ".png" in f])
    imgs_left = natsorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if ".jpg" in f])
    imgs_right = natsorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if ".jpg" in f])

    for f_left, f_right in zip(imgs_left, imgs_right):
        img_left = cv2.imread(f_left, cv2.IMREAD_GRAYSCALE)
        img_right = cv2.imread(f_right, cv2.IMREAD_GRAYSCALE)

        # # Important!!: downsample images
        # img_left = cv2.resize(img_left, None, fx=downsample_ratio, fy=downsample_ratio, interpolation=cv2.INTER_AREA)
        # img_right = cv2.resize(img_right, None, fx=downsample_ratio, fy=downsample_ratio, interpolation=cv2.INTER_AREA)

        dst_left = cv2.undistort(img_left, M_l, d_l, None)
        dst_right = cv2.undistort(img_right, M_r, d_r, None)

        dst = np.concatenate([dst_left, dst_right], axis=1)

        # plt.imshow(dst)
        plt.show()


def sensitivity(data_dir, iter_num):
    l_fx = np.zeros(iter_num)
    l_fy = np.zeros(iter_num)
    for i in range(iter_num):
        M_l, M_r, T = stereoCalibration(data_dir, 1*i, test_mode=True)
        l_fx[i] = M_l[0, 0]
        l_fy[i] = M_l[1, 1]
    np.save(data_dir+'/l_fx_list', l_fx)
    np.save(data_dir+'/l_fy_list', l_fy)
    mean_lx = l_fx.mean()
    std_lx = l_fx.std()
    mean_ly = l_fy.mean()
    std_ly = l_fy.std()
    print('mean l_x focal:', mean_lx, '\nstd l_x focal:', std_lx)
    print('mean l_y focal:', mean_ly, '\nstd l_y focal:', std_ly)


def CharucoCalibration(imgs_left, imgs_right):

    from cv2 import aruco

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
    ids_left_all = []
    ids_right_all = []
    image_size = None  # Determined at runtime

    objp = np.zeros((1, (CHARUCOBOARD_ROWCOUNT-1) * (CHARUCOBOARD_COLCOUNT-1), 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHARUCOBOARD_ROWCOUNT-1, 0:CHARUCOBOARD_COLCOUNT-1].T.reshape(-1, 2) * 3.0  # 3mm board

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
        if response_l == response_r and response_l == (CHARUCOBOARD_ROWCOUNT-1) * (CHARUCOBOARD_COLCOUNT-1):
            print('charuco corners found')
            objpoints.append(objp)
            # Add these corners and ids to our calibration arrays
            imgpoints_left.append(charuco_corners_left)
            ids_left_all.append(charuco_ids_left)
            imgpoints_right.append(charuco_corners_right)
            ids_right_all.append(charuco_ids_right)

            # Draw the Charuco board we've detected to show our calibrator the board was properly detected
            img_l = aruco.drawDetectedCornersCharuco(
                image=img_l,
                charucoCorners=charuco_corners_left,
                charucoIds=charuco_ids_left)
            img_r = aruco.drawDetectedCornersCharuco(
                image=img_r,
                charucoCorners=charuco_corners_right,
                charucoIds=charuco_ids_right)

            # If our image size is unknown, set it now
            if not image_size:
                image_size = gray_left.shape[::-1]

            # Reproportion the image, maxing width or height at 1000
            proportion = max(img_l.shape) / 1000.0
            img_l = cv2.resize(img_l, (int(img_l.shape[1] / proportion), int(img_l.shape[0] / proportion)))
            proportion = max(img_r.shape) / 1000.0
            img_r = cv2.resize(img_r, (int(img_r.shape[1] / proportion), int(img_r.shape[0] / proportion)))

            # Pause to display each image, waiting for key press
            img = np.hstack((img_l, img_r))
            cv2.imshow('Charuco board', img)
            # cv2.imshow('Charuco board', img_r)
            cv2.waitKey(0)
        else:
            print("Not able to detect a charuco board in image: {}".format(f_left[:-4]))
            os.remove(f_left)
            os.remove(f_right)

    # Destroy any open CV windows
    cv2.destroyAllWindows()

    # Make sure at least one image was found
    if len(f_left) < 1:
        # Calibration failed because there were no images, warn the user
        print(
            "Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
        # Exit for failure
        exit()

    # Make sure we were able to calibrate on at least one charucoboard by checking
    # if we ever determined the image size
    if not image_size:
        # Calibration failed because we didn't see any charucoboards of the PatternSize used
        print(
            "Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
        # Exit for failure
        exit()

    # Now that we've seen all of our images, perform the camera calibration
    # based on the set of points we've discovered
    ret_left, M_left, d_left, r_left, t_left, _, _, perViewErrors_left = aruco.calibrateCameraCharucoExtended(
        charucoCorners=imgpoints_left,
        charucoIds=ids_left_all,
        board=CHARUCO_BOARD,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=cv2.CALIB_FIX_PRINCIPAL_POINT)

    ret_right, M_right, d_right, r_right, t_right, _, _, perViewErrors_right = aruco.calibrateCameraCharucoExtended(
        charucoCorners=imgpoints_right,
        charucoIds=ids_right_all,
        board=CHARUCO_BOARD,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=cv2.CALIB_FIX_PRINCIPAL_POINT)

    # Print reprojection error to the console
    print('RPE left:', perViewErrors_left.mean(), '\n', 'RPE right:', perViewErrors_right.mean())

    # stereo flags
    flags = 0
    flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    flags |= cv2.CALIB_SAME_FOCAL_LENGTH
    flags |= cv2.CALIB_ZERO_TANGENT_DIST
    flags |= cv2.CALIB_FIX_K1
    flags |= cv2.CALIB_FIX_K2
    flags |= cv2.CALIB_FIX_K3
    flags |= cv2.CALIB_FIX_K4
    # flags |= cv2.CALIB_FIX_K5

    stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                            cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    ret, M_l, d_l, M_r, d_r, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, M_left,
                                                              d_left, M_right, d_right, image_size,
                                                              criteria=stereocalib_criteria, flags=flags)

    print("stereo param:")
    print('ret:\n', ret)
    print('M_l:\n', M_l)
    print('d_l:\n', d_l)
    print('M_r:\n', M_r)
    print('d_r:\n', d_r)
    print('R:\n', R)
    print('T:\n', T)

    # if test_mode:
    #     return M_l, M_r, T
    # else:
    #     save_dir = outParams_dir
    #     np.save(save_dir + '/M_l', M_l)
    #     np.save(save_dir + '/d_l', d_l)
    #     np.save(save_dir + '/M_r', M_r)
    #     np.save(save_dir + '/d_r', d_r)
    #     np.save(save_dir + '/R', R)
    #     np.save(save_dir + '/T', T)


if __name__ == '__main__':
    # stereoCalibration('AXXB_Good_large_motion_414', rm_num=0, test_mode=False)
    # sensitivity('AXXB_Good_large_motion_414', 5)
    # l_fx_list = np.load('AXXB_Good_large_motion_414'+'/l_fx_list.npy')
    # l_fy_list = np.load('AXXB_Good_large_motion_414' + '/l_fy_list.npy')
    #
    # print('mean l_x focal:', l_fx_list.mean(), '\nstd l_x focal:', l_fx_list.std())
    # print('mean l_y focal:', l_fy_list.mean(), '\nstd l_y focal:', l_fy_list.std())
    # sns.distplot(l_fy_list, bins=5, hist=True, kde=True, rug=False,
    #              hist_kws={"color": "steelblue"}, kde_kws={"color": "purple"})
    # plt.show()

    # read image
    inputImgs_dir = 'Charuco_Calib_419'
    left_dir = inputImgs_dir + "/calib_left"
    right_dir = inputImgs_dir + "/calib_right"

    imgs_left = natsorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if ".jpeg" in f])
    imgs_right = natsorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if ".jpeg" in f])
    CharucoCalibration(imgs_left, imgs_right)

