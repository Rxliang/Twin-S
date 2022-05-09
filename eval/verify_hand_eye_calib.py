import shutil

import numpy as np
from dataLoader import dataLoader
from Solver import solver
import os
from natsort import natsorted
import cv2
from stereo_vision_pipeline import getChessPoses, Q_intrinsic, draw, draw_full, chessboard_pose
ld = dataLoader()
sol = solver()


def get_init_frame(file_idx_list, c2m_path, num_idx, h2e_path):

    pose_idx = file_idx_list[num_idx]
    cam_poses = ld.getRealPose(pose_idx, f'{calib_dir}/fwd_pose_camhand.csv')
    _, cam_poses = sol.seven2trans(cam_poses)
    camhand2cam = np.load(h2e_path)
    cam2marker = np.load(c2m_path)
    Op2marker = cam_poses@camhand2cam@cam2marker[:, :, num_idx]
    return Op2marker


def propagate(file_idx_list, calib_dir, h2e_path, num_idx):

    Op2marker = get_init_frame(file_idx_list, f'{calib_dir}/A_cam2marker.npy', num_idx, h2e_path)
    camhand2cam = np.load(h2e_path)
    marker2Op = sol.invTransformation(Op2marker)

    cam2marker_list = []
    for idx in file_idx_list:
        cam_poses_temp = ld.getRealPose(idx, f'{calib_dir}/fwd_pose_camhand.csv')
        _, cam_pose = sol.seven2trans(cam_poses_temp)
        cam2marker = marker2Op@cam_pose@camhand2cam
        cam2marker_list.append(cam2marker)
    return cam2marker_list


def backProject(file_idx_list, intrinsic_params_dir, cam2marker_list, calib_dir, org_imgpts_list, pattern, visual=True):
    data_dir = calib_dir + '/rectified_left'
    reproj_dir = calib_dir + '/reprojections'
    shutil.rmtree(reproj_dir)
    os.makedirs(reproj_dir, exist_ok=True)
    cam_mtx = Q_intrinsic(calib_dir)
    cam_dist = np.load(intrinsic_params_dir + '/d_l.npy')
    cam_dist = np.zeros([5])
    j = 0
    bp_error = []
    for idx in file_idx_list:
        marker2cam = sol.invTransformation(cam2marker_list[j])
        R = marker2cam[:3, :3]
        t = marker2cam[:3, 3]
        rvecs, _ = cv2.Rodrigues(R)
        tvecs = t
        width = 2
        axis = np.float32([[0, 0, 0], [1.5 * width, 0, 0], [0, 1.5 * width, 0], [0, 0, 1.5 * width]]).reshape(-1, 3)

        # imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts, jac = cv2.projectPoints(pattern, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        imgpts_org = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts_org = np.int32(imgpts_org[0]).reshape(-1, 2)

        # back project error
        bp_error.append(np.linalg.norm(imgpts[0] - org_imgpts_list[j+47]))
        j += 1
        if visual:
            img = cv2.imread(os.path.join(data_dir, str(idx+47) + '_axis.png'))

            img = draw(img, imgpts_org, color='org')
            # img = draw_full(img, imgpts, (7, 10), color='b')
            img = draw_full(img, imgpts, (12, 9), color='p')
            # cv2.imwrite(os.path.join(data_dir, str(idx) + '_axis_verify_1.png'), img)
            cv2.imwrite(os.path.join(reproj_dir, str(idx) + '_reproj.png'), img)
    print('Mean backProj error:', np.mean(bp_error))
    return np.mean(bp_error)

if __name__ == '__main__':
    calib_dir = 'AXXB_Good_small_motion_419'
    intrinsic_dir = 'intrinsic_params_421'
    checkerboard_pattern = np.load('checkerboard_pattern.npy')
    charuco_pattern = np.load('charuco_pattern.npy')
    h2e_path = f'{calib_dir}/hand_eye_X_axxb.npy'  # axxb

    file_idx_list = np.load(f'{calib_dir}/file_idx_list.npy')[:500]
    org_imgpts_list = np.load(f'{calib_dir}/imgpts_list.npy')[:600]

    print(len(file_idx_list))
    cam2marker_list = propagate(file_idx_list, calib_dir, h2e_path, 0)[:600]
    RPE = backProject(file_idx_list, intrinsic_dir, cam2marker_list, calib_dir, org_imgpts_list, charuco_pattern,
                visual=True)
    # RPE_list = []
    # for i in range(len(file_idx_list)):
    #     cam2marker_list = propagate(file_idx_list, calib_dir, h2e_path, i)
    #     RPE = backProject(file_idx_list, intrinsic_dir, cam2marker_list, calib_dir, org_imgpts_list, checkerboard_pattern,
    #                 visual=True)
    #     RPE_list.append(RPE)
    # print('Mean RPE for all:', np.mean(RPE_list))

    # charuco_pattern = np.zeros([108, 3])
    # count = 0
    # for i in range(12):
    #     for j in range(9):
    #         charuco_pattern[count] = np.array([i*3,j*3,0])
    #         count += 1
    # # print(charuco_pattern)
    # charuco_pattern = np.save('charuco_pattern.npy', charuco_pattern)