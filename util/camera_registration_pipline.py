import numpy as np
import cv2
from natsort import natsorted
import os
import re
import open3d as o3d
import copy
import pandas as pd
import matplotlib.pyplot as plt
from Solver import  solver
sol = solver()

# # cv2.imshow('imgray', imgray)
#
# ret, thresh = cv2.threshold(imgray, 127, 255, 0)
# contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# # print(hierarchy.shape)
# imgnew = cv2.drawContours(img, contours, -1, (0,255,0), 3)
# # cv2.imshow('img_org', imgnew)


def getCircleCentroid(dir, img_idx):
    CircleCentroid = {}
    idx = ['left', 'right']
    image = []
    count = 0
    center = []
    for i in idx:
        img = cv2.imread(f'microscope_calibration_2.8/{dir + i}/{img_idx}.jpeg') #220 40
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

def stereoMatch(img_idx):
    left_dir = 'microscope_calibration_2.8/dst_left'
    right_dir = 'microscope_calibration_2.8/dst_right'
    imgs_left = natsorted([os.path.join(left_dir, f) for f in os.listdir(left_dir) if ".jpeg" in f])
    imgs_right = natsorted([os.path.join(right_dir, f) for f in os.listdir(right_dir) if ".jpeg" in f])


    imgL = cv2.imread(f'microscope_calibration_2.8/dst_left/{img_idx}.jpeg', cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(f'microscope_calibration_2.8/dst_right/{img_idx}.jpeg', cv2.IMREAD_GRAYSCALE)
    # get image index
    # idx = re.findall('\d+', str(f_left))[2]

    # disparity range tuning
    window_size = 3
    min_disp = 0
    num_disp = 320 - min_disp

    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=416,  # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=8,
        P1=0,#8 * 3 * window_size ** 2,
        # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        P2=0,#32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=0,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_HH
    )
    disparity = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    # print(disparity[312, 1318])
    plt.imshow(disparity, 'gray')
    plt.show()

    return disparity

def get3Dcoord(coord_disp_list):
    M_l = np.load('stereo_calibration_data/M_l.npy')
    T = np.load('stereo_calibration_data/T.npy')
    Q = np.load('stereo_calibration_data/Q.npy')
    f_length = M_l[0, 0]
    baseline = abs(T[0])
    cx = M_l[0, 2]
    cy = M_l[1, 2]
    Z = (f_length * baseline) / (coord_disp_list[2]) # depth
    X = (coord_disp_list[0][0] - cx) * Z / f_length
    Y = (coord_disp_list[0][1] - cy) * Z / f_length
    # coord = cv2.reprojectImageTo3D(dis_map, Q)
    coord = np.array([X, Y, Z])
    return coord

def get3DcoordForClick(tuple):
    '''
    get 3D points for click.
    '''
    M_l = np.load('stereo_calibration_data/M_l.npy')
    T = np.load('stereo_calibration_data/T.npy')
    Q = np.load('stereo_calibration_data/Q.npy')
    f_length = M_l[0, 0]
    baseline = abs(T[0])
    cx = M_l[0, 2]
    cy = M_l[1, 2]
    Z = (f_length * baseline) / (tuple[1]) # depth
    X = (tuple[0][0] - cx) * Z / f_length
    Y = (tuple[0][1] - cy) * Z / f_length
    # coord = cv2.reprojectImageTo3D(dis_map, Q)
    coord = np.array([X, Y, Z])
    print('3dcoord:', coord)
    return coord

def find_NN_for_disparity(dict_l, dict_r):
    from sklearn.neighbors import NearestNeighbors
    arr_r = (np.array(list(dict_r.values()))+np.array([400, 0]))
    arr_l = np.array(list(dict_l.values()))
    arr = np.vstack((arr_l, arr_r))
    neigh = NearestNeighbors(n_neighbors=2, radius=1)
    neigh.fit(arr)
    distances, indices = neigh.kneighbors(arr)
    indices = indices+1
    # print(indices[:][::-1])
    print(indices[indices[::-1] != indices])
    idx = indices[indices[::-1] != indices]
    idx = idx[0:int(len(idx)/2)]
    dict_center = {**dict_l, **dict_r}
    disparity = []
    for i in list(range(len(idx)))[::2]:
        # print(str(idx[i]))
        l_coordinates = dict_center[str(idx[i])]
        r_coordinates = dict_center[str(idx[i+1])]
        dis_tempt = abs(l_coordinates[0]-r_coordinates[0])
        # print(zip(l_coordinates,r_coordinates,dis_tempt))
        disparity.append([l_coordinates,r_coordinates,dis_tempt,str(idx[i])+' in left image'])
    # for i in idx:
    #     dict_center[i]
    # print(neigh.kneighbors_graph(arr).toarray())
    return disparity

def getRealPose(idx):
    df = pd.read_csv('microscope_calibration_2.8/fwd_pose_drill.csv')
    pose_x = df['pose.position.x'][idx] * 1000
    pose_y = df['pose.position.y'][idx] * 1000
    pose_z = df['pose.position.z'][idx] * 1000
    orin_x = df['pose.orientation.x'][idx]
    orin_y = df['pose.orientation.y'][idx]
    orin_z = df['pose.orientation.z'][idx]
    orin_w = df['pose.orientation.w'][idx]
    pose = np.array([pose_x, pose_y, pose_z])
    pg = np.load('../pivotTool_geometry.npy')
    real_pose = pose + pg
    # print(real_pose, '\n', pose)
    return real_pose


def on_EVENT_LBUTTONDOWN(event, x, y, flags, record):
    '''
    click the matched points on left image and remember the
    sequence. Use that exact sequence to click again on geometry.
    '''
    global reg_data_lst
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        coord = np.array([x, y])
        for i in range(len(disparity)):
            dis = np.linalg.norm(np.array(disparity[i][0]) - coord)
            if dis <= 100:
                cv2.circle(img, (x, y), 1, (0, 0, 255), thickness=-1)
                cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                            1.0, (0, 0, 0), thickness=1)
                cv2.imshow("image", img)
                # print(coord)
                record.append((disparity[i][0], disparity[i][2]))
            else:
                continue
    if event == cv2.EVENT_RBUTTONDOWN:
        cv2.putText(img, 'Record Saved!', (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 255), thickness=3)
        cv2.imshow("image", img)
        record = list(set(record))
        print('record:', record)
        coords3d = []
        for i in record:
            coords3d.append(get3DcoordForClick(i))
        coords3d = np.hstack(coords3d).T
        print('3D coords:', coords3d)
        reg_data_lst.append(coords3d)
        # np.save('matched_points_3D', coords3d)

def pickPointsInGeom(geometry_points):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")
    geometry_pcd = o3d.geometry.PointCloud()
    geometry_pcd.points = o3d.utility.Vector3dVector(geometry_points)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(geometry_pcd)
    vis.run()  # user picks points

    vis.destroy_window()
    print("Selected:")
    print(vis.get_picked_points())
    # print(type(vis.get_picked_points()))
    point_cloud_array = np.asarray(geometry_pcd.points)
    coordinates = point_cloud_array[vis.get_picked_points()]
    return coordinates

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom= 5,
                                      front=[100,   15.16825071,  400],
                                      lookat=[221.70334021, 15.80506897, 1440],
                                      up=[221.70334021, 15.80506897, 800.07384863])

def mannual_registration(geom_coordinates, camera_points):

    atracsys_pcd = o3d.geometry.PointCloud()
    atracsys_pcd.points = o3d.utility.Vector3dVector(geom_coordinates)

    camera_pcd = o3d.geometry.PointCloud()
    camera_pcd.points = o3d.utility.Vector3dVector(camera_points)
    # o3d.io.write_point_cloud("./camera.pcd", pcd)
    # source = o3d.io.read_point_cloud("./camera.pcd")
    # corr = np.array([[0,0],[1,1]])
    corr  = np.repeat(np.array(list(range(len(camera_points)))),2).reshape(len(camera_points),2)
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_mtx = p2p.compute_transformation(atracsys_pcd, camera_pcd,
                                            o3d.utility.Vector2iVector(corr))
    # visulize the result
    draw_registration_result(atracsys_pcd, camera_pcd, trans_mtx)

    # Arun registration
    # F = sol.solvePointCloudReg_Arun(camera_points, geom_coordinates)
    # F = np.hstack(F)
    # F = np.vstack((F, np.array([0, 0, 0, 1])))
    # print('F_Arun:', F)
    F = p2p.compute_transformation(camera_pcd, atracsys_pcd,
                                            o3d.utility.Vector2iVector(corr))
    # draw_registration_result(camera_pcd, atracsys_pcd, F)
    FF_inv_R = trans_mtx[:3, :3]@F[:3, :3]
    trans_mtx_t = trans_mtx[:3, 3].reshape(3, 1)
    F_t = F[:3, 3].reshape(3, 1)
    FF_inv_p = trans_mtx[:3, :3]@F_t + trans_mtx_t
    print('FF_inv\n',FF_inv_R,'\n',FF_inv_p)

    return trans_mtx #From Atracsys -> Camera 3d Coordinates


###########################################################################

# Circle the marker
img_idx = 175
# img_idx_list = [15, 30, 35, 175, 201, 220]
img_idx_list = [85]
reg_data_lst = [] #save in data for registartion
geom_data_list = []


for i in img_idx_list:
    record = []  # save points picked
    c_l, c_r, img = getCircleCentroid('dst_', i)
    disparity = find_NN_for_disparity(c_l, c_r)
    # print('disp', disparity)
    real_geometry_points = getRealPose(i)
    # print('real Pose', real_geometry_points)
    cv2.namedWindow("image")
    a = cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN, param=record)
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    geom_data_list.append(pickPointsInGeom(real_geometry_points))

reg_data_lst = np.vstack(reg_data_lst)
geom_data_list = np.vstack(geom_data_list)
print('reg_lst:\n',reg_data_lst,'\n','geom_list:\n',geom_data_list)

np.save('camera_reg_geom_data_npy/reg_data_lst', reg_data_lst)
np.save('camera_reg_geom_data_npy/geom_data_list', geom_data_list)
# mannual registration

camera_points = np.load('matched_points_3D.npy')
trans_mtx = mannual_registration(geom_data_list, reg_data_lst)
print('transformation:\n', trans_mtx)

dis = np.linalg.norm(trans_mtx[:3, 3])
print('dis from op to camera:', dis)

