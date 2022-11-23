import shutil
import sys
import argparse
from glob import glob
sys.path.append('../')
sys.path.insert(0, '/home/shc/RoboMaster/util')
import numpy as np
from dataLoader import dataLoader
from Solver import solver
import os
from natsort import natsorted
import cv2
import pickle
from preparation.stereo_vision_pipeline import getChessPoses, draw, draw_full, chessboard_pose

ld = dataLoader()
sol = solver()


def initialization():
    global file , args
    
    parser = argparse.ArgumentParser(description="Reproject charuco pattern to verify the X.")
    parser.add_argument("--handeye_dir", dest="X_path", help="path of handeye transformation X", default='hand_eye_X.npy', type=str)
    parser.add_argument('--outdir',dest="output_dir", help="Output directory.", required=True, type=str)
    parser.add_argument('--cam_mtx_dir',dest="cam_mtx_dir", default='../params/zed_M_l.npy', help="camera matrix", type=str)
    
    args = parser.parse_args()

    limg_path = args.output_dir +'/extract_poses/limg/'

    if not os.path.exists(args.output_dir):
            os.makedirs(args.output_dir)
            print("Create Folder at Designated Address...")

    if not os.path.exists(limg_path):
        os.makedirs(limg_path)
        print("No Designated Address limg found...")
    json_dir = args.output_dir + '/'
    ld.loadHandeyeJson(json_dir)

def get_init_frame(aligned_camhand_path, aligned_campose_path, num_idx, h2e_path):
    B2H_seven = ld.getToolPose(num_idx, aligned_camhand_path)
    W2E_seven = ld.getToolPose(num_idx, aligned_campose_path)
    _, B2H = sol.seven2trans(B2H_seven)
    _, W2E = sol.seven2trans(W2E_seven)
    H2E = np.load(h2e_path)
    E2W = sol.invTransformation(W2E)
    # optical tracker to Marker
    B2W = B2H@H2E@E2W
    return B2W


def propagate(aligned_camhand_path, aligned_campose_path, timestamp_list, h2e_path, num_idx):

    B2W = get_init_frame(aligned_camhand_path, aligned_campose_path, num_idx, h2e_path)
    H2E = np.load(h2e_path)
    W2B = sol.invTransformation(B2W)
    W2E_list = []
    for (idx, timestamp) in enumerate(timestamp_list):
        B2H_seven = ld.getToolPose(idx, aligned_camhand_path)
        _, B2H = sol.seven2trans(B2H_seven)
        W2E = sol.invTransformation(W2B@B2H@H2E)
        W2E_list.append(W2E)
    return W2E_list


def backProject(timestamp_list, intrinsic_params_dir, W2E_list, org_imgpts_dict, pattern, visual=True):
    global args
    img_dir = args.output_dir + '/extract_poses/limg'
    reproj_dir = args.output_dir + '/reprojections'
    if os.path.exists(reproj_dir):
        shutil.rmtree(reproj_dir)
    os.makedirs(reproj_dir, exist_ok=True)
    cam_mtx = np.load(intrinsic_params_dir)
    cam_dist = np.zeros([5])
    j = 0
    bp_error = []

    # cv record video
    fps = 30
    size = (1920, 1080)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    saveName = os.path.join(reproj_dir, 'reprojection.avi')
    videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)


    for (idx,timestamp) in enumerate(timestamp_list):
        W2E = W2E_list[j]
        R = W2E[:3, :3]
        t = W2E[:3, 3]
        rvecs, _ = cv2.Rodrigues(R)
        tvecs = t
        width = 12
        axis = np.float32([[0, 0, 0], [1.5 * width, 0, 0], [0, 1.5 * width, 0], [0, 0, 1.5 * width]]).reshape(-1, 3)

        # imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts, jac = cv2.projectPoints(pattern, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        imgpts_org = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts_org = np.int32(imgpts_org[0]).reshape(-1, 2)

        # back project error
        try:
            real_idx = idx
            print('idx=', real_idx, 'J=', j)
            print(np.linalg.norm(imgpts[0] - org_imgpts_dict[str(timestamp_list[real_idx])]))
            bp_error.append(np.linalg.norm(imgpts[0] - org_imgpts_dict[str(timestamp_list[real_idx])]))
            j += 1
            if visual:
                img = cv2.imread(os.path.join(img_dir, str(timestamp_list[real_idx]) + '.png'))

                img = draw(img, imgpts_org, color='org')
                # img = draw_full(img, imgpts, (7, 10), color='b')
                img = draw_full(img, imgpts, (8, 11), color='p')
                # cv2.imwrite(os.path.join(reproj_dir, str(j) + '_reproj.png'), img)
                videoWriter.write(img)
        except:
            j += 1
            continue

    videoWriter.release()
    print('Mean reProjection error:', np.mean(bp_error))
    return np.mean(bp_error)

if __name__ == '__main__':
    initialization()
    # sol.make_charuco_pattern('../params/zed_charuco_pattern.npy')

    calib_dir = args.output_dir
    intrinsic_dir = args.cam_mtx_dir
    # checkerboard_pattern = np.load('../params/checkerboard_pattern.npy')
    charuco_pattern = np.load('../params/zed_charuco_pattern.npy')
    h2e_path = args.X_path  #

    aligned_camhand_path = f'{calib_dir}/aligned_tf_poses_camhand.csv'
    aligned_campose_path = f'{calib_dir}/aligned_cam_pose.csv'
    timestamp_list = ld.getTimestamp_list(aligned_campose_path)
    with open(f'{calib_dir}/imgpts_dict.pickle', 'rb') as handle:
        org_imgpts_dict = pickle.load(handle)
    
    W2E_list = propagate(aligned_camhand_path, aligned_campose_path, timestamp_list, h2e_path, 0)
    RPE = backProject(timestamp_list, intrinsic_dir, W2E_list, org_imgpts_dict, charuco_pattern,
                visual=True)
    
    
    
    # RPE_list = []
    # for i in range(len(file_idx_list)):
    #     cam2marker_list = propagate(file_idx_list, calib_dir, h2e_path, i)
    #     RPE = backProject(file_idx_list, intrinsic_dir, cam2marker_list, calib_dir, org_imgpts_list, checkerboard_pattern,
    #                 visual=True)
    #     RPE_list.append(RPE)
    # print('Mean RPE for all:', np.mean(RPE_list))
