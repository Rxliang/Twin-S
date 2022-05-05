# Detecting chessboard in the data
# Johns Hopkins University
# Date: Apr 09, 2022

from __future__ import print_function
import os
import re
import cv2
import shutil
import numpy as np
from Solver import solver
from dataLoader import dataLoader
from natsort import natsorted
from rectifyStereo import rectifyStereo
from calibrate_stereo_chess import stereoCalibration
import sys
from cv2 import aruco
from dq_calibration import handeye_calib_dq
import matplotlib as plt
sol = solver()


def draw(img, imgpts, color='org'):
    """
    Draw the detction result (axis & origin of teh chessboard) on the image
    """

    if color == 'p':
        c = [240, 32, 160]
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]), c, 3)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]), c, 3)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]), c, 3)
        print('draw!')
    elif color == 'b':
        c = [255, 144, 30]
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]), c, 3)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]), c, 3)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]), c, 3)
    elif color == 'org':
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[1]),[0,0,255],3)  #BGR
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],3)
        cv2.line(img, tuple(imgpts[0]), tuple(imgpts[3]),[255,0,0],3)
    return img


def draw_full(img, imgpts, pattern_size, color):
    """
    Draw full checkerboard on the image
    """
    if color == 'p':
        c = [240, 32, 160]
    elif color == 'b':
        c = [255, 144, 30]
    for j in range(pattern_size[0]):
        for k in range(pattern_size[1]):
            if k+1 < pattern_size[1]:
                cv2.line(img, tuple(imgpts[pattern_size[1]*j + k]), tuple(imgpts[pattern_size[1]*j + k+1]), c, 2)
                if j + 1 < pattern_size[0]:
                    cv2.line(img, tuple(imgpts[pattern_size[1] * j + k]), tuple(imgpts[pattern_size[1] * (j+1) + k]), c,2)
    cv2.line(img, tuple(imgpts[pattern_size[1]-1]), tuple(imgpts[-1]), c, 2)

    # for j in range(pattern_size[0]):
    #     for k in range(pattern_size[1]):
    #         if k+1 < pattern_size[1]:
    #             cv2.line(img, tuple(imgpts[pattern_size[1]*j + k]), tuple(imgpts[pattern_size[1]*j + k+1]), c, 2)
    #             if j + 1 < pattern_size[0]:
    #                 cv2.line(img, tuple(imgpts[pattern_size[1] * j + k]), tuple(imgpts[pattern_size[1] * (j+1) + k]), c,2)
    # cv2.line(img, tuple(imgpts[pattern_size[1]-1]), tuple(imgpts[-1]), c, 2)
    return img


def pose_inv(pose):
    """
    Inverse of a homogenenous transformation.
    Args:
    - pose (4x4 numpy array)
    Return:
    - inv_pose (4x4 numpy array)
    """
    R = pose[:3, :3]
    t = pose[:3, 3]

    inv_R = R.T
    inv_t = - np.dot(inv_R, t)

    inv_pose = np.c_[inv_R, np.transpose(inv_t)]
    inv_pose = np.r_[inv_pose, [[0, 0, 0, 1]]]

    return inv_pose

def luckyPick(interval, ori_dir, tar_dir):

    if not os.path.exists(tar_dir):
        os.makedirs(tar_dir)
    if os.listdir(tar_dir):
        shutil.rmtree(tar_dir)
        os.makedirs(tar_dir)

    file_list = os.listdir(ori_dir)
    item_list = list(range(0, len(file_list), interval))
    for file in item_list:
        file_path = str(file) + '.jpeg'
        move_item = os.path.join(ori_dir, file_path)
        shutil.copy(move_item, tar_dir)

def chessboard_pose(img_dir, img_filename, cam_mtx, cam_dist, objp, width, pattern=(7, 10)):
    """
    Find the chessboard pose with OpenCV.

    @type  img_dir: string
    @param img_dir: directory of the image
    @type  img_filename: string
    @param img_filename: filename of the image
    @type  cam_mtx: numpy.ndarray
    @param cam_mtx: intrinsic matrix of the camera
    @type  cam_dist: numpy.ndarry
    @param cam_dist: distortion coefficient of the camera
    @type  objp: numpy.ndarray
    @param objp: 3D positions of the points on the chessboard
    @type  pattern: tuple
    @param pattern: pattern of the chessboard
    """
    img_filepath = os.path.join(img_dir, img_filename)
    img_name = img_filename.split('.')[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0005)
    chessboard_size_tuple = pattern
    
    # IR and RGB both read as RGB and then turned to gray
    img = cv2.imread(img_filepath)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size_tuple, None)

    if ret == True:
        # Increase corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        cv2.drawChessboardCorners(img, chessboard_size_tuple, corners, ret)
        cv2.imwrite(os.path.join(img_dir, img_name + "_corner.png"), img)

        _, rvecs, tvecs, inlier = cv2.solvePnPRansac(objp, corners2, cam_mtx, cam_dist)
        R, _ = cv2.Rodrigues(rvecs)

        axis = np.float32([[0, 0, 0], [3*width,0,0], [0,3*width,0], [0,0,3*width]]).reshape(-1, 3)
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)

        imgpts = np.int32(imgpts).reshape(-1, 2)
        # print (imgpts)
        img = draw(img, imgpts, color='org')
        cv2.imwrite(os.path.join(img_dir, img_name + '_axis.png'), img)

        print("Finish processing: {}".format(img_filename))

        return R, tvecs, imgpts[0]
    else:
        os.remove(img_filepath)
        print("Cannot find chessboard in {}".format(img_filename))
        return None, None, None


def Charuco_pose(img_dir, img_filename, cam_mtx, cam_dist, width, pattern):

    cam_dist = np.zeros([5])
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)

    # Create constants to be passed into OpenCV and Aruco methods
    CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=pattern[0]+1,
        squaresY=pattern[1]+1,
        squareLength=3,
        markerLength=2,
        dictionary=ARUCO_DICT)

    img_filepath = os.path.join(img_dir, img_filename)
    img_name = img_filename.split('.')[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0005)

    # IR and RGB both read as RGB and then turned to gray
    img = cv2.imread(img_filepath)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    param = aruco.DetectorParameters_create()
    param.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    # Find aruco markers in the query image
    corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=param)

    # Outline the aruco markers found in our query image
    im_with_charuco_board = aruco.drawDetectedMarkers(image=img, corners=corners)

    # Get charuco corners and ids from detected aruco markers
    response, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=gray,
        board=CHARUCO_BOARD)

    # if response != pattern[0]*pattern[1]:
    #     os.remove(img_filepath)
    #     return None, None, None
    # Refine
    charucoCorners = cv2.cornerSubPix(gray, charucoCorners, (11, 11), (-1, -1), criteria)
    # Draw corners
    im_with_charuco_board = aruco.drawDetectedCornersCharuco(img.copy(), charucoCorners, charucoIds)

    # Find Pose

    # rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.02, cam_mtx, cam_dist)
    # for i in range(len(tvecs)):
    #     aruco.drawAxis(im_with_charuco_board, cam_mtx, cam_dist, rvecs[i], tvecs[i], 0.02 / 2)
    # cv2.imshow('d', im_with_charuco_board)
    # cv2.waitKey(0)

    retval, rvecs, tvecs = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, CHARUCO_BOARD, cam_mtx,
                                                          cam_dist, None, None) # posture estimation from a charuco board
    axis = np.float32([[0, 0, 0], [3 * width, 0, 0], [0, 3 * width, 0], [0, 0, 3 * width]]).reshape(-1, 3)
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # print(imgpts)
    # im_with_charuco_board = draw(im_with_charuco_board, imgpts, color='org')
    if retval:
        im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, cam_mtx, cam_dist, rvecs, tvecs, 2*3)
        cv2.imwrite(os.path.join(img_dir, img_name + '_axis.png'), im_with_charuco_board)
        print("Finish processing: {}".format(img_filename))
        R, _ = cv2.Rodrigues(rvecs)
        return R, tvecs, imgpts[0]
    else:
        os.remove(img_filepath)
        print("Cannot find charuco in {}".format(img_filename))
        return None, None, None


def getChessPoses(data_dir, cam_mtx, cam_dist, charuco):
    if not charuco:
        pattern = (7, 10)
        width = 2
    else:
        pattern = (10, 7)
        width = 3

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    
    for i in range(pattern[1]):
        for j in range(pattern[0]):
            index = i * pattern[0] + j
            objp[index, 0] = j * width
            objp[index, 1] = i * width
            objp[index, 2] = 0

    file_list = os.listdir(data_dir)
    file_list = natsorted(file_list)
    poses = []
    imgpts_list = []
    cam_dist = np.zeros([5])
    for fname in file_list:
        if ".jpeg" in fname:
            if not charuco:
                R, t, imgpts_org = chessboard_pose(data_dir, fname, cam_mtx, cam_dist, objp, width, pattern)
            else:
                R, t, imgpts_org = Charuco_pose(data_dir, fname, cam_mtx, cam_dist, width, pattern)
            if R is None:
                continue
            # print(R, '\n', t)
            # break
            imgpts_list.append(imgpts_org)
            # pose = make_rigid_transformation(t, R)
            homo_mat = np.c_[R, np.reshape(t, (3, 1))]
            pose = np.r_[homo_mat, [[0, 0, 0, 1]]]
            # pose = pose_inv(pose)
            poses.append(pose)
    return poses, imgpts_list


def getHandEyeAB(calib_dir, robot_poses, marker_poses):
    '''
    Get A and B of AXXB problem
    '''
    assert len(robot_poses) == len(marker_poses), 'robot poses and marker poses are not of the same length!'
    n = len(robot_poses)
    pose_inds = np.arange(n)
    # np.random.shuffle(pose_inds)

    print("Total Pose: %i" % n)
    camera_to_marker = np.zeros((4, 4, n))
    base_to_hand = np.zeros((4, 4, n))

    for i in range(n):
        camera_to_marker[:, :, i] = marker_poses[pose_inds[i]]
        base_to_hand[:, :, i] = robot_poses[pose_inds[i]]

    np.save(f'{calib_dir}/A_cam2marker', camera_to_marker)
    np.save(f'{calib_dir}/B_base2hand', base_to_hand)


def axxb(calib_dir, robot_poses, marker_poses):
    """
    AX=XB solver.

    @rtype  calib_pose: 4x4 numpy.ndarry
    @return calib_pose: target poses.
    """
    assert len(robot_poses) == len(marker_poses), 'robot poses and marker poses are not of the same length!'
    n = len(robot_poses)
    pose_inds = np.arange(n)
    # np.random.shuffle(pose_inds)

    print("Total Pose: %i" % n)
    camera_to_marker = np.zeros((4, 4, n))
    base_to_hand = np.zeros((4, 4, n))
    A = np.zeros((4, 4, n-1))
    B = np.zeros((4, 4, n-1))
    alpha = np.zeros((3, n-1))
    beta = np.zeros((3, n-1))

    M = np.zeros((3, 3))
    nan_num = 0
    for i in range(n-1):
        camera_to_marker[:, :, i] = marker_poses[pose_inds[i]]
        base_to_hand[:, :, i] = robot_poses[pose_inds[i]]
        A[:, :, i] = np.matmul(sol.invTransformation(robot_poses[pose_inds[i+1]]), robot_poses[pose_inds[i]])
        B[:, :, i] = np.matmul(marker_poses[pose_inds[i+1]], sol.invTransformation(marker_poses[pose_inds[i]]))

        alpha[:, i] = sol.get_mat_log(A[:3, :3, i])
        beta[:, i] = sol.get_mat_log(B[:3, :3, i])
        M += np.outer(beta[:, i], alpha[:, i])

        # Bad pair of transformation are very close in the orientation.
        # They will give nan result
        if np.sum(np.isnan(alpha[:, i])) + np.sum(np.isnan(beta[:, i])) > 0:
            nan_num += 1
            continue
        else:
            M += np.outer(beta[:, i], alpha[:, i])
    # don`t forget the last pose
    camera_to_marker[:, :, n-1] = marker_poses[pose_inds[n-1]]
    base_to_hand[:, :, n-1] = robot_poses[pose_inds[n-1]]
    np.save(f'{calib_dir}/A_cam2marker', camera_to_marker)
    np.save(f'{calib_dir}/B_base2hand', base_to_hand)
    print('nan_num:', nan_num)

    # Get the rotation matrix
    mtm = np.matmul(M.T, M)
    u_mtm, s_mtm, vh_mtm = np.linalg.svd(mtm)
    R = np.matmul(np.matmul(np.matmul(u_mtm, np.diag(np.power(s_mtm, -0.5))), vh_mtm), M.T)

    # Get the tranlation vector
    I_Ra_Left = np.zeros((3*(n-1), 3))
    ta_Rtb_Right = np.zeros((3 * (n-1), 1))
    for i in range(n-1):
        I_Ra_Left[(3*i):(3*(i+1)), :] = np.eye(3) - A[:3, :3, i]
        ta_Rtb_Right[(3*i):(3*(i+1)), :] = np.reshape(A[:3, 3, i] - np.dot(R, B[:3, 3, i]), (3, 1))


    t = np.linalg.lstsq(I_Ra_Left, ta_Rtb_Right, rcond=-1)[0]

    calib_pose = np.c_[R, t]
    calib_pose = np.r_[calib_pose, [[0, 0, 0, 1]]]

    print("Calibration Result:\n", calib_pose)

    return calib_pose

def Q_intrinsic(intrinsic_params_dir):
    Q = np.load(f'{intrinsic_params_dir}/Q.npy')
    fl = Q[2, 3]
    bl = 1 / Q[3, 2]
    cx = -Q[0, 3]
    cy = -Q[1, 3]
    new_intrinsic = np.eye(3)
    new_intrinsic[0, 0] = fl
    new_intrinsic[0, 2] = cx
    new_intrinsic[1, 2] = cy
    new_intrinsic[1, 1] = fl
    # print('baseLine', bl)
    return new_intrinsic

def verifyAxxB(idx):

    def getPanel(dirPath):
        fileNum = 0
        p_list = []
        for lst in os.listdir(dirPath):
            sub_path = os.path.join(dirPath, lst)
            # print(sub_path)
            if lst == 'hmd':
                for name in os.listdir(sub_path):
                    name = os.path.join(os.path.abspath(sub_path), name)
                    data = ld.loadJson(name)
                    p_list.append(data)
                    fileNum = fileNum + 1
                p_list = np.vstack(p_list)
        # print('point num:', fileNum)
        return p_list

    # get pan to phacon
    pan = getPanel('../run_2.21_281_points_cloud')
    pan = np.mean(pan, 0)
    _, Trac2Pan_Reg = sol.seven2trans(pan)
    phacon2Trac_reg = np.load('../Phacon2Op_2.21.npy')
    phacon2Pan = phacon2Trac_reg @ Trac2Pan_Reg
    Pan2phacon = sol.invTransformation(phacon2Pan)
    # np.save('../Pan2phacon.npy', Pan2phacon)

    Trac2Camhand = ld.getRealPose(idx, '../GT_CSV_2.21/fwd_pose_camhand.csv')
    _, Trac2Camhand = sol.seven2trans(Trac2Camhand)
    Trac2Pan = ld.getRealPose(idx, '../GT_CSV_2.21/fwd_pose_pan.csv')
    _, Trac2Pan = sol.seven2trans(Trac2Pan)

    Trac2phacon = Trac2Pan@Pan2phacon
    phacon2Cam = np.load(f'verify_axxb_phacon2Cam/{idx}.npy')
    Camhand2Trac = sol.invTransformation(Trac2Camhand)
    inference_X = Camhand2Trac@Trac2phacon@phacon2Cam
    print('dis infer_X L2:', np.linalg.norm(inference_X[:3, 3]))

    X = np.load('hand_eye_X_37.npy')
    c2ph = sol.invTransformation(X)@Camhand2Trac@Trac2phacon
    dis_cam2Phacon_use_X = np.linalg.norm(c2ph[:3, 3])
    print('dis X L2:', np.linalg.norm(X[:3, 3]))
    print('F_inference(phacon2Cam)@ invF:\n', phacon2Cam@c2ph)
    np.save('infer_X', inference_X)
    return inference_X

def saveMat_intrinsic(intrin_dir):
    from scipy.io import loadmat
    save_path = f'{intrin_dir}'
    np.save(f'{save_path}/M_l', loadmat(f'{save_path}/M_l.mat')['M_l'])
    np.save(f'{save_path}/M_r', loadmat(f'{save_path}/M_r.mat')['M_r'])
    np.save(f'{save_path}/d_l', loadmat(f'{save_path}/d_l.mat')['d_l'])
    np.save(f'{save_path}/d_r', loadmat(f'{save_path}/d_r.mat')['d_r'])
    np.save(f'{save_path}/R', loadmat(f'{save_path}/R.mat')['R'])
    np.save(f'{save_path}/T', loadmat(f'{save_path}/T.mat')['T'])

def main(interval, calib_dir, intrinsic_dir, handeyeMode, charuco=False):

    ori_dir = [f'{calib_dir}/limg', f'{calib_dir}/rimg']
    tar_dir = [f'{calib_dir}/prepare_left', f'{calib_dir}/prepare_right']

    # image pick interval
    for i in range(2):
        luckyPick(interval, ori_dir[i], tar_dir[i])
        print(' Finished copy!')

    # rectifyStereo(calib_dir, intrinsic_dir, leftOnly=False)

    # test_vibr = []
    robot_poses = []
    data_dir = calib_dir + '/rectified_left'
    cam_mtx = Q_intrinsic(calib_dir)
    cam_dist = np.load(intrinsic_dir + '/d_l.npy')
    cam_dist = np.zeros([5])
    marker_poses, imgpts_list = getChessPoses(data_dir, cam_mtx, cam_dist, charuco)
    # save right up points on chessboard
    np.save(f'{calib_dir}/imgpts_list', imgpts_list)

    # file_list = os.listdir(data_dir)
    # file_list = natsorted(file_list)
    # file_idx_list = []
    # for file in file_list:
    #     if file.endswith('axis.png'):
    #         i = int(file[:-9])
    #         print(i)
    #         file_idx_list.append(i)
    #         cam_poses_temp = ld.getRealPose(i, f'{calib_dir}/fwd_pose_camhand.csv')
    #         # tracker base
    #         _, cam_poses_temp = sol.seven2trans(cam_poses_temp)
    #         robot_poses.append(cam_poses_temp)
    #         np.save(f'{calib_dir}/file_idx_list', file_idx_list)

            # # use pan as base
            # pan_poses_temp = ld.getRealPose(i, f'{calib_dir}/fwd_pose_pan.csv')
            # _, pan_poses_temp = sol.seven2trans(pan_poses_temp)
            # _, cam_poses_temp = sol.seven2trans(cam_poses_temp)
            # robot_poses_temp = np.linalg.inv(pan_poses_temp) @ cam_poses_temp
            # robot_poses.append(robot_poses_temp)

    # Pose from JSON
    file_idx_list = []
    file_list = os.listdir(data_dir)
    for file in file_list:
        if file.endswith('axis.png'):
            idx = int(file[:-9])
            file_idx_list.append(idx)
    camhand_poses_files = os.listdir(f'{calib_dir}/pose_camera')
    camhand_poses_files = natsorted(camhand_poses_files)
    cam_hand_poses = []
    for num, file in enumerate(camhand_poses_files):
        if num in file_idx_list:
            cam_poses_temp = ld.loadJson(f'{calib_dir}/pose_camera/{file}')
            cam_hand_poses.append(cam_poses_temp)
            # tracker base
            _, cam_poses_temp = sol.seven2trans(cam_poses_temp)
            robot_poses.append(cam_poses_temp)
    np.save(f'{calib_dir}/file_idx_list', file_idx_list)

    if handeyeMode == 1:
        # using ETH hand-eye calibration
        getHandEyeAB(calib_dir, robot_poses, marker_poses)
        A = np.load(f'{calib_dir}/A_cam2marker.npy')
        B = np.load(f'{calib_dir}/B_base2hand.npy')
        X = handeye_calib_dq.hand_eye_calibration(A, B)

    elif handeyeMode == 2:
        # using traditional hand-eye calibration
        X = axxb(calib_dir, robot_poses, marker_poses)
    np.save(f'{calib_dir}/hand_eye_X_axxb', X)
    # X = np.load(f'{calib_dir}/hand_eye_X_axxb.npy')
    print('translation:', np.linalg.norm(X[:3, 3]))

if __name__ == '__main__':
    ld = dataLoader()

    # # directory of calibration images and target path
    calib_dir = 'Charuco_handeye_425'
    intrinsic_dir = 'intrinsic_params_421'
    main(1, calib_dir, intrinsic_dir, handeyeMode=1, charuco=True)

    # A = np.load(f'{calib_dir}/A_cam2marker.npy')
    # B = np.load(f'{calib_dir}/B_base2hand.npy')
    # # A = A[:, :, :15]
    # # B = B[:, :, :15]
    # X = handeye_calib_dq.hand_eye_calibration(A, B)
    # np.save(f'{calib_dir}/hand_eye_X_axxb', X)