from glob import glob
import rosbag
import rospy
import cv2
import numpy as np
import os
import time
import argparse
import pickle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from cv2 import aruco
from tf_to_csv import write_transformation_to_csv_file

bridge = CvBridge()


def verify_cv_bridge():
    arr = np.zeros([480, 640])
    msg = bridge.cv2_to_imgmsg(arr)
    try:
        bridge.imgmsg_to_cv2(msg)
    except ImportError:
        # log.log(logging.WARNING, "libcv_bridge.so: cannot open shared object file. Please source ros env first.")
        return False

    return True

def initialization():
    global file , args
    
    parser = argparse.ArgumentParser(description="Extract images from rosbag.")
    # parser.add_argument("img_dir", help="Input image directory", default='./test.bag', type=str)
    parser.add_argument('--bag',dest="bag_file", help="Input ROS bag directory", default='./test.bag', type=str)
    parser.add_argument('--outdir',dest="output_dir", help="Output directory.", default='~/Desktop/data', type=str)
    parser.add_argument('--stereo',dest="stereo", help="Stereo or monocular.", default=True, type=bool)
    # parser.add_argument('--topic',dest="img_topic", help="Topic of image.", default='/camera/color/image_raw/compressed', type=str)
    parser.add_argument('--l_topic',dest="l_img_topic", help="Topic of left image.", default='/zedm/zed_node/left/image_rect_color/compressed', type=str)
    # parser.add_argument('--r_topic',dest="r_img_topic", help="Topic of right image.", default='/fwd_rimage/compressed', type=str)
    parser.add_argument('--csv',dest='csv_file_name', help='Path to output csv file', default='./', type=str)
    # parser.add_argument('--csv_i',dest='csv_file_name_inv', required=True, help='Path to output inverse csv pose file')
    args = parser.parse_args()

    args.output_dir = args.bag_file[:-4] + '/extract_poses/'
    args.csv_file_name = args.bag_file[:-4] + '/cam_pose.csv'
    valid = verify_cv_bridge()
    # verify_ROS_connection()
    limg_path = args.output_dir +'/limg/'
    # rimg_path = args.output_dir +'/rimg/'
    # img_path = args.output_dir +'/img/'
    if valid:
        if not os.path.exists(args.output_dir):
                os.makedirs(args.output_dir)
                print("Create Folder at Designated Address...")
        if args.stereo:
            if not os.path.exists(limg_path):
                os.makedirs(limg_path)
                print("Create Folder at Designated Address limg...")
            # if not os.path.exists(rimg_path):
            #     os.makedirs(rimg_path)
                print("Create Folder at Designated Address rimg...")
            time_str = time.strftime("%Y%m%d_%H%M%S")
            # file = h5py.File(args.output_dir + '/' + time_str + ".hdf5", "w")
        # else:
        #     # if not os.path.exists(img_path):
        #     #     os.makedirs(img_path)
        #         print("Create Folder at Designated Address img...")
        #Under Construction
    else:
        print("Failed")


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

def chessboard_pose(img, timestamp, cam_mtx, cam_dist, objp, width, pattern=(7, 10)):
    """
    """
    global args
    # img_filepath = os.path.join(img_dir, img_filename)
    # img_name = img_filename.split('.')[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0005)
    chessboard_size_tuple = pattern
    
    # IR and RGB both read as RGB and then turned to gray
    # img = cv2.imread(img_filepath)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size_tuple, None)

    if ret == True:
        # Increase corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        cv2.drawChessboardCorners(img, chessboard_size_tuple, corners, ret)
        cv2.imwrite(os.path.join(args.output_dir +'/limg/', timestamp + "_corner.png"), img)

        _, rvecs, tvecs, inlier = cv2.solvePnPRansac(objp, corners2, cam_mtx, cam_dist)
        R, _ = cv2.Rodrigues(rvecs)

        axis = np.float32([[0, 0, 0], [3*width,0,0], [0,3*width,0], [0,0,3*width]]).reshape(-1, 3)
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)

        imgpts = np.int32(imgpts).reshape(-1, 2)
        # print (imgpts)
        img = draw(img, imgpts, color='org')
        cv2.imwrite(os.path.join(args.output_dir +'/limg/', timestamp + '_axis.png'), img)

        print("Finish processing: ", timestamp)

        return R, tvecs, imgpts[0]
    else:
        # os.remove(img_filepath)
        print("Cannot find chessboard in {}".format(timestamp))
        return None, None, None


def Charuco_pose(img, timestamp, cam_mtx, cam_dist, width, pattern):
    global args
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)
    
    # plt.imshow(img)
    # plt.show()
    # print(img.shape)
    # Create constants to be passed into OpenCV and Aruco methods
    CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=pattern[0]+1,
        squaresY=pattern[1]+1,
        squareLength=15,
        markerLength=12,
        dictionary=ARUCO_DICT)

    # img_filepath = os.path.join(img_dir, img_filename)
    # img_name = img_filename.split('.')[0]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0005)

    # IR and RGB both read as RGB and then turned to gray
    # img = cv2.imread(img_filepath)
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
    if retval:                                                
        axis = np.float32([[0, 0, 0], [3 * width, 0, 0], [0, 3 * width, 0], [0, 0, 3 * width]]).reshape(-1, 3)
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
        imgpts = np.int32(imgpts).reshape(-1, 2)

    # if retval:
        im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, cam_mtx, cam_dist, rvecs, tvecs, 12*3)
        cv2.imwrite(os.path.join(args.output_dir +'/limg/', str(timestamp) + '.png'), im_with_charuco_board)
        print("Finish processing: ", timestamp)
        R, _ = cv2.Rodrigues(rvecs)
        return R, tvecs, imgpts[0]
    else:
        # os.remove(img_filepath)
        print("Cannot find charuco in: ", timestamp)
        return None, None, None


def getChessPoses(img, timestamp, cam_mtx, cam_dist, charuco, pattern=(7, 10), width=2):
    # if not charuco:
    #     pattern = (7, 10)
    #     width = 2
    # else:
    #     pattern = (10, 7)
    #     width = 3

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)

    for i in range(pattern[1]):
        for j in range(pattern[0]):
            index = i * pattern[0] + j
            objp[index, 0] = j * width
            objp[index, 1] = i * width
            objp[index, 2] = 0

    # file_list = os.listdir(data_dir)
    # file_list = natsorted(file_list)
    poses = []
    cam_dist = np.zeros([5])
    # for fname in file_list:

    if not charuco:
        R, t, imgpts_org = chessboard_pose(img, timestamp, cam_mtx, cam_dist, objp, width, pattern)
    else:
        R, t, imgpts_org = Charuco_pose(img, timestamp, cam_mtx, cam_dist, width, pattern)
    if R is None:
        return
    # print(R, '\n', t)
    # break


    r = Rot.from_matrix(R)
    quat = r.as_quat()
    # print('t', t)
    return t, quat, imgpts_org


def extractPose(args, cam_mtx, cam_dist):
    bridge = CvBridge()
    count = 0
    csv_file = open(args.csv_file_name, 'w')
    # inv_csv_file = open(args.csv_file_name_inv, 'w')

    start = time.time()
    imgpts_dict = {}
    with rosbag.Bag(args.bag_file, 'r') as bag:
        if args.stereo:
            start = time.time()
            for topic, msg, t in bag.read_messages(topics = args.l_img_topic):
                img_sec = '{:.5f}'.format(msg.header.stamp.to_sec())
                
                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except CvBridgeError as e:
                    print(e)
                try:
                    t, quat, imgpts_org = getChessPoses(cv_image, img_sec, cam_mtx, cam_dist, charuco=True, pattern=(10,7), width=15)
                except:
                    continue
                 
                imgpts_dict[img_sec] = imgpts_org
                
                trans = np.identity(4)
                trans[:3, 3] = t.flatten()
                r = Rot.from_quat(quat)
                R = r.as_matrix()
                trans[:3 ,:3] = R
                inv_trans = pose_inv(trans)
                inv_t = inv_trans[:3, 3].reshape([3, 1])
                inv_R = inv_trans[:3, :3]
                inv_r = Rot.from_matrix(inv_R)
                inv_quat = inv_r.as_quat()

                # save a inverse matrix (C2W)
                # inv_csv_file.write(
                # str(img_sec) + ', ' +
                # str(t[0,0]/1000) + ', ' + str(t[1,0]/1000) + ', ' +
                # str(t[2,0]/1000) + ', ' + str(quat[0]) + ', ' +
                # str(quat[1]) + ', ' + str(quat[2]) + ', ' +
                # str(quat[3]) + '\n')

                # save a inverse matrix (W2C)

                csv_file.write(
                str(img_sec) + ', ' +
                str(inv_t[0,0]/1000) + ', ' + str(inv_t[1,0]/1000) + ', ' +
                str(inv_t[2,0]/1000) + ', ' + str(inv_quat[0]) + ', ' +
                str(inv_quat[1]) + ', ' + str(inv_quat[2]) + ', ' +
                str(inv_quat[3]) + '\n')

                count += 1
                print(count)

            # count = 0
            # for topic, msg, t in bag.read_messages(topics = args.r_img_topic):
            #     img_sec = msg.header.stamp.to_sec()
            #     img_sec_list_l.append(img_sec)

            #     try:
            #         cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
            #     except CvBridgeError as e:
            #         print(e)
            #     image_path = args.output_dir +'/rimg/' + str(count) + '.jpeg'
            #     pil_image = Image.fromarray(cv_image[:,:,::-1])
            #     pil_image.save(image_path)
            #     count += 1
            #     print(count)

            end = time.time()
            print(end - start)
            print("Complete Saving")
            with open(f'{args.output_dir}/../imgpts_dict.pickle', 'wb') as handle:
                pickle.dump(imgpts_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
            return imgpts_dict

        # else:
        #     for topic, msg, t in bag.read_messages(topics = args.img_topic):
        #         img_sec = msg.header.stamp.nsecs
        #         img_sec_list.append(img_sec)

        #         try:
        #             cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #         except CvBridgeError as e:
        #             print(e)
        #         image_path = args.output_dir +'/img/' + str(count) + '.jpeg'
        #         pil_image = Image.fromarray(cv_image[:,:,::-1])
        #         pil_image.save(image_path)
        #         count += 1
        #         print(count)

        #     end = time.time()
        #     print(end - start)
        #     print("Complete Saving")
            # return img_sec_list

def main():
    global args
    initialization()
    # img_sec_list = bag2images(args)
    # np.save('time_stamp',img_sec_list)
    target_frame = '/atracsys/Camera_hand/measured_cp'
    csv_name = args.bag_file[:-4] + '/tf_poses_camhand.csv'
    write_transformation_to_csv_file(args.bag_file, target_frame,
                                     csv_name)
    data_dir = args.output_dir +'/limg/'
    cam_mtx = np.load('cam_mtx.npy')
    # print(cam_mtx)
    cam_dist = np.zeros([5])
    extractPose(args, cam_mtx, cam_dist)



if __name__ == '__main__':
    main()
    # time_stamp = np.load('time_stamp.npy')
    # print(len(time_stamp))
