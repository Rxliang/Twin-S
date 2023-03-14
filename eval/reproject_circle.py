<<<<<<< HEAD:eval/main.py
import os
import re
import cv2
import shutil
import numpy as np
from Solver import solver
from dataLoader import dataLoader
from stereo_vision_pipeline import getChessPoses, Q_intrinsic, draw, draw_full
from natsort import natsorted
from rectifyStereo import rectifyStereo
from calibrate_stereo_chess import stereoCalibration
import sys
from cv2 import aruco
from dq_calibration import handeye_calib_dq
import matplotlib as plt
import pandas as pd
sol = solver()
ld = dataLoader()


def img2video(path, frame, fps, size):

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    saveName = path[2:-2] + '.avi'
    videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)

    num_files = len(os.listdir(path))
    print(num_files)
    videoWriter.write(frame)
    videoWriter.release()


def general_Test(drill_path, cam_hand_path, img_path, X_path):
    import cv2

    print("Begin General Test")
    ################################################################ Get Panel to Phacon Registration Result
    # pan = getPanel('run_2.21_281_points_cloud')
    # pan = np.mean(pan, 0)
    # _, Op2Pan_reg = sol.seven2trans(pan)
    # phacon2Op = np.load('Phacon2Op_2.21.npy')
    img_list = os.listdir(img_path)
    img_list = natsorted(img_list)

    drill_df = pd.read_csv(drill_path)
    cam_hand_df = pd.read_csv(cam_hand_path)
    cam_mtx = Q_intrinsic(path)
    # print(cam_mtx)
    # cam_mtx[2, 2] = cam_mtx[2, 2] - 300
    # cam_mtx = np.load("intrinsic_params_510/M_l.npy")
    X = np.load(X_path)
    fps = 30
    size = (1920, 1080)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    saveName = os.path.join(path, 'demo.avi')
    videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)

    for i in range(len(drill_df)):
        Op2Drill = ld.getToolPose(i, drill_df)  # B_W
        # _, Op2Pan_data = sol.seven2trans(p_1)
        Op2Camhand = ld.getToolPose(i, cam_hand_df)  # B_H
        img_path_tmp = os.path.join(img_path, img_list[i])
        tempt_image = cv2.imread(img_path_tmp)
        _, Op2Drill = sol.seven2trans(Op2Drill)
        _, Op2Camhand = sol.seven2trans(Op2Camhand)
        Cam2Drill = sol.invTransformation(Op2Camhand @ X) @ Op2Drill
        Drill2Cam = sol.invTransformation(Cam2Drill)
        Cam2Op = sol.invTransformation(Op2Camhand @ X)

        p = np.array([61.63730898,  221.16114215, 1288.60447745])
        # print('p', np.linalg.norm(p))
        c2t = Cam2Op[:3, :3] @ p.T + Cam2Op[:3, 3]

        # print('c2t', c2t, np.linalg.norm(c2t))
        R = Drill2Cam[:3, :3]
        t = Drill2Cam[:3, 3]
        R_cd = Cam2Drill[:3, :3]
        t_cd = Cam2Drill[:3, 3]

        pivot0 = np.array([[-27.48950654, 212.28827629,  60.06926844]]).T
        # print('Op2tip',Op2Drill[:3,:3] @ pivot0 + Op2Drill[:3, 3], p)
        # pivot = np.array([[12.50173232,  6.24592626,  279.6911268]]).T
        c2tip = R_cd @ pivot0 + np.expand_dims(t_cd, 1)

        pix,val,dist = projectWithoutDistortion(cam_mtx, 1920, 1080, c2tip)
        # print(pix,val,dist)
        imgpt = np.int32(pix).reshape(-1, 2)
        # print('imgpt:', imgpt[0])
        imgpt[0][0] = imgpt[0][0] - 370
        imgpt[0][1] = imgpt[0][1] + 80
        showimg = cv2.circle(tempt_image, imgpt[0], 45, [0, 0, 255], thickness=10)

        videoWriter.write(showimg)

        # cv2.imshow('piont', showimg)
        # cv2.waitKey(0)
        # if i == 10:
        #     break
    videoWriter.release()

def projectWithoutDistortion(intrinsic_matrix, width, height, pts):
    """
    Projects a list of points to the camera defined transform, intrinsics and distortion
    :param intrinsic_matrix: 3x3 intrinsic camera matrix
    :param width: the image width
    :param height: the image height
    :param pts: a list of point coordinates (in the camera frame) with the following format
    :return: a list of pixel coordinates with the same lenght as pts
    """

    _, n_pts = pts.shape

    # Project the 3D points in the camera's frame to image pixels without considering the distorcion
    # From https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    pixs = np.zeros((2, n_pts), dtype=np.float)

    # fx, _, cx, _, fy, cy, _, _, _ = intrinsic_matrix

    fx = intrinsic_matrix[0, 0]
    fy = intrinsic_matrix[1, 1]
    cx = intrinsic_matrix[0, 2]
    cy = intrinsic_matrix[1, 2]

    x = pts[0, :]
    y = pts[1, :]
    z = pts[2, :]

    dists = np.linalg.norm(pts[0:3, :], axis=0)  # compute distances from point to camera
    xl = np.divide(x, z)  # compute homogeneous coordinates
    yl = np.divide(y, z)  # compute homogeneous coordinates

    pixs[0, :] = fx * xl + cx
    pixs[1, :] = fy * yl + cy
    # Compute mask of valid projections
    valid_z = z > 0
    valid_xpix = np.logical_and(pixs[0, :] >= 0, pixs[0, :] < width)
    valid_ypix = np.logical_and(pixs[1, :] >= 0, pixs[1, :] < height)
    valid_pixs = np.logical_and(valid_z, np.logical_and(valid_xpix, valid_ypix))
    return pixs, valid_pixs, dists


def getCircleCentroid(dir, img_idx):
    CircleCentroid = {}
    idx = ['left', 'right']
    image = []
    count = 0
    center = []
    for i in idx:
        img = cv2.imread(os.path.join(dir, f"rectified_{i}", f"{img_idx}.jpeg")) #220 40
        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 120, param1=100, param2=30, minRadius=0, maxRadius=0)
        # print('circles',type(circles),circles.shape)  #circles <class 'numpy.ndarray'> (1, 3, 3)
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            #绘制圆 (i[0],i[1])为圆心，i[2]为半径
            cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
            #绘制圆心
            cv2.circle(img, (i[0], i[1]), 2, (255, 0, 0), 3)
            centertext = "(" + str(i[0]) + ", " + str(i[1]) + ")"
            idxtest = f'{count + 1}'
            cv2.putText(img, centertext, (i[0]-100, i[1]-20), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 1, 8, 0)
            cv2.putText(img, idxtest, (i[0], i[1] - 40), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 1, 8, 0)
            CircleCentroid[f'{count+1}'] = (i[0], i[1])
            count += 1

        image.append(img)
        center.append(CircleCentroid)
        CircleCentroid = {}
        # count=0

    rec = np.concatenate([image[0], image[1]], axis=1)
    # print(rec.shape)
    rec_org = rec
    rec = cv2.resize(rec, (2200, 600))
    cv2.imshow('circles', rec)
    cv2.waitKey()
    cv2.destroyAllWindows()
    return center[0], center[1], rec_org


# def get3Dcoord(coord_disp_list, path):
#     M_l = Q_intrinsic(path)
#     T = np.load(os.path.join('intrinsic_params_421', 'T.npy'))
#     f_length = M_l[0, 0]
#     baseline = abs(T[0])
#     cx = M_l[0, 2]
#     cy = M_l[1, 2]
#     Z = (f_length * baseline) / (coord_list[0][0]-coord_list[1][0])# depth
#     X = (coord_disp_list[0][0] - cx) * Z / f_length
#     Y = (coord_disp_list[0][1] - cy) * Z / f_length
#     # coord = cv2.reprojectImageTo3D(dis_map, Q)
#     coord = np.array([X, Y, Z])
#     return coord


if __name__ == '__main__':
    path = '../Drill_510_0'
    intrinsic_dir = '/intrinsic_params_510'
    handeye_dir = 'Charuco_handeye_510'
    # center0, center1, _ = getCircleCentroid(path, 1)
    # coord_list = [center0['1'], center1['2']]
    # coord = get3Dcoord(coord_list, path)
    # print(coord, np.linalg.norm(coord))
    drill_path = os.path.join(path, 'fwd_pose_drill.csv')
    cam_hand_path = os.path.join(path, 'fwd_pose_camhand.csv')
    X_path = os.path.join(handeye_dir, 'hand_eye_X_axxb.npy')
    img_path = os.path.join(path, 'rectified_left')
=======
import os
import re
import cv2
import shutil
import numpy as np
import sys
sys.path.insert(0, '/home/shc/Twin-S/util')
from Solver import solver
from dataLoader import dataLoader
# from stereo_vision_pipeline import getChessPoses, Q_intrinsic, draw, draw_full
from natsort import natsorted
# from rectifyStereo import rectifyStereo
# from calibrate_stereo_chess import stereoCalibration
from cv2 import aruco
# from dq_calibration import handeye_calib_dq
import matplotlib as plt
import pandas as pd
sol = solver()
ld = dataLoader()


def img2video(path, frame, fps, size):

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    saveName = path[2:-2] + '.avi'
    videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)

    num_files = len(os.listdir(path))
    print(num_files)
    videoWriter.write(frame)
    videoWriter.release()


def general_Test(drill_path, cam_hand_path, img_path, X_path):
    import cv2

    print("Begin General Test")
    ################################################################ Get Panel to Phacon Registration Result
    # pan = getPanel('run_2.21_281_points_cloud')
    # pan = np.mean(pan, 0)
    # _, Op2Pan_reg = sol.seven2trans(pan)
    # phacon2Op = np.load('Phacon2Op_2.21.npy')
    img_list = os.listdir(img_path)
    img_list = natsorted(img_list)

    drill_df = pd.read_csv(drill_path)
    # cam_hand_df = pd.read_csv(cam_hand_path)
    cam_mtx = np.load('../util/cam_mtx.npy')
    # print(cam_mtx)
    # cam_mtx[2, 2] = cam_mtx[2, 2] - 300
    # cam_mtx = np.load("intrinsic_params_510/M_l.npy")
    X = np.load(X_path)
    fps = 30
    size = (1920, 1080)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    saveName = os.path.join(path, 'demo.avi')
    videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)

    for i in range(len(drill_df)):
        Op2Drill = ld.getRealPose(i, drill_path)  # B_W
        # _, Op2Pan_data = sol.seven2trans(p_1)
        Op2Camhand = ld.getRealPose(i, cam_hand_path)  # B_H
        img_path_tmp = os.path.join(img_path, img_list[i])
        tempt_image = cv2.imread(img_path_tmp)
        _, Op2Drill = sol.seven2trans(Op2Drill)
        _, Op2Camhand = sol.seven2trans(Op2Camhand)
        Cam2Drill = sol.invTransformation(Op2Camhand @ X) @ Op2Drill
        Drill2Cam = sol.invTransformation(Cam2Drill)
        Cam2Op = sol.invTransformation(Op2Camhand @ X)

        # p = np.array([61.63730898,  221.16114215, 1288.60447745])
        # print('p', np.linalg.norm(p))
        # c2t = Cam2Op[:3, :3] @ p.T + Cam2Op[:3, 3]

        # print('c2t', c2t, np.linalg.norm(c2t))
        R = Drill2Cam[:3, :3]
        t = Drill2Cam[:3, 3]
        R_cd = Cam2Drill[:3, :3]
        t_cd = Cam2Drill[:3, 3]

        pivot0 = np.load('../params/t_tip.npy')
        # pivot0 = np.array([[12.83037253], [-168.75504173], [56.3511996]]) #1017
        # print('Op2tip',Op2Drill[:3,:3] @ pivot0 + Op2Drill[:3, 3], p)
        # pivot = np.array([[12.50173232,  6.24592626,  279.6911268]]).T
        c2tip = R_cd @ pivot0 + np.expand_dims(t_cd, 1)

        R_d2tip =np.array([[-0.18795203,  0.06529684, -0.98000528],
                        [ 0.97997203, -0.05437232, -0.19156843],
                        [-0.06579397, -0.99638345, -0.05376969]])

        cam2tip = np.vstack([np.hstack([Cam2Drill[:3,:3]@R_d2tip,c2tip.reshape(3,1)]),np.array([0,0,0,1]).reshape(1,4)])

        # optimize the cam2tip
        T_delta = np.array([[ 0.9999,  0.0109, -0.0023, 0.008],
                            [-0.0110,  0.9988, -0.0482, 0.0085],
                            [ 0.0018,  0.0482,  0.9988, -0.0073],
                            [0, 0, 0, 1]])
        T_delta[:3, 3] = T_delta[:3, 3]*1000
        T_delta = np.identity(4)
        cam2tip = T_delta@cam2tip
        
        cam2tip_t = cam2tip[:3, 3].reshape(3,1)
        pix,val,dist = projectWithoutDistortion(cam_mtx, 1920, 1080, cam2tip_t)
        # print(pix,val,dist)
        imgpt = np.int32(pix).reshape(-1, 2)
        # print('imgpt:', imgpt[0])
        imgpt[0][0] = imgpt[0][0]
        imgpt[0][1] = imgpt[0][1]
        tip_point = (imgpt[0][0], imgpt[0][1])
        showimg = cv2.circle(tempt_image, tip_point, 15, [0, 0, 255], thickness=5)

        videoWriter.write(showimg)

        # cv2.imshow('piont', showimg)
        # cv2.waitKey(0)
        # if i == 10:
        #     break
    videoWriter.release()

def projectWithoutDistortion(intrinsic_matrix, width, height, pts):
    """
    Projects a list of points to the camera defined transform, intrinsics and distortion
    :param intrinsic_matrix: 3x3 intrinsic camera matrix
    :param width: the image width
    :param height: the image height
    :param pts: a list of point coordinates (in the camera frame) with the following format
    :return: a list of pixel coordinates with the same lenght as pts
    """

    _, n_pts = pts.shape

    # Project the 3D points in the camera's frame to image pixels without considering the distorcion
    # From https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    pixs = np.zeros((2, n_pts), dtype=float)

    # fx, _, cx, _, fy, cy, _, _, _ = intrinsic_matrix

    fx = intrinsic_matrix[0, 0]
    fy = intrinsic_matrix[1, 1]
    cx = intrinsic_matrix[0, 2]
    cy = intrinsic_matrix[1, 2]

    x = pts[0, :]
    y = pts[1, :]
    z = pts[2, :]

    dists = np.linalg.norm(pts[0:3, :], axis=0)  # compute distances from point to camera
    xl = np.divide(x, z)  # compute homogeneous coordinates
    yl = np.divide(y, z)  # compute homogeneous coordinates

    pixs[0, :] = fx * xl + cx
    pixs[1, :] = fy * yl + cy
    # Compute mask of valid projections
    valid_z = z > 0
    valid_xpix = np.logical_and(pixs[0, :] >= 0, pixs[0, :] < width)
    valid_ypix = np.logical_and(pixs[1, :] >= 0, pixs[1, :] < height)
    valid_pixs = np.logical_and(valid_z, np.logical_and(valid_xpix, valid_ypix))
    return pixs, valid_pixs, dists


def getCircleCentroid(dir, img_idx):
    CircleCentroid = {}
    idx = ['left', 'right']
    image = []
    count = 0
    center = []
    for i in idx:
        img = cv2.imread(os.path.join(dir, f"rectified_{i}", f"{img_idx}.jpeg")) #220 40
        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(imgray, cv2.HOUGH_GRADIENT, 1, 120, param1=100, param2=30, minRadius=0, maxRadius=0)
        # print('circles',type(circles),circles.shape)  #circles <class 'numpy.ndarray'> (1, 3, 3)
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            #绘制圆 (i[0],i[1])为圆心，i[2]为半径
            cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
            #绘制圆心
            cv2.circle(img, (i[0], i[1]), 2, (255, 0, 0), 3)
            centertext = "(" + str(i[0]) + ", " + str(i[1]) + ")"
            idxtest = f'{count + 1}'
            cv2.putText(img, centertext, (i[0]-100, i[1]-20), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 1, 8, 0)
            cv2.putText(img, idxtest, (i[0], i[1] - 40), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 1, 8, 0)
            CircleCentroid[f'{count+1}'] = (i[0], i[1])
            count += 1

        image.append(img)
        center.append(CircleCentroid)
        CircleCentroid = {}
        # count=0

    rec = np.concatenate([image[0], image[1]], axis=1)
    # print(rec.shape)
    rec_org = rec
    rec = cv2.resize(rec, (2200, 600))
    cv2.imshow('circles', rec)
    cv2.waitKey()
    cv2.destroyAllWindows()
    return center[0], center[1], rec_org


if __name__ == '__main__':
    path = sys.argv[1]#'../Drill_510_0'
    handeye_dir = sys.argv[2]#'Charuco_handeye_510'

    drill_path = os.path.join(path, 'fwd_pose_drill.csv')
    cam_hand_path = os.path.join(path, 'fwd_pose_camhand.csv')
    X_path = os.path.join(handeye_dir)
    img_path = os.path.join(path, 'limg')
>>>>>>> 06eb023fb3325be98551eb4925691df9cffd2f84:eval/reproject_circle.py
    general_Test(drill_path, cam_hand_path, img_path, X_path)