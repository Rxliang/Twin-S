from glob import glob
import rosbag
import rospy
import cv2
import numpy as np
import os
import time
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
from cv2 import aruco

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
    parser.add_argument('--csv',dest='csv_file_name', required=True, help='Path to output csv file')
    args = parser.parse_args()
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

# def luckyPick(interval, ori_dir, tar_dir):

#     if not os.path.exists(tar_dir):
#         os.makedirs(tar_dir)
#     if os.listdir(tar_dir):
#         shutil.rmtree(tar_dir)
#         os.makedirs(tar_dir)

#     file_list = os.listdir(ori_dir)
#     item_list = list(range(0, len(file_list), interval))
#     for file in item_list:
#         file_path = str(file) + '.jpeg'
#         move_item = os.path.join(ori_dir, file_path)
#         shutil.copy(move_item, tar_dir)

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

        print("Finish processing: {}".format(timestamp))

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
    axis = np.float32([[0, 0, 0], [3 * width, 0, 0], [0, 3 * width, 0], [0, 0, 3 * width]]).reshape(-1, 3)
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, cam_mtx, cam_dist)
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # print(imgpts)
    # im_with_charuco_board = draw(im_with_charuco_board, imgpts, color='org')
    if retval:
        im_with_charuco_board = aruco.drawAxis(im_with_charuco_board, cam_mtx, cam_dist, rvecs, tvecs, 2*3)
        cv2.imwrite(os.path.join(args.output_dir +'/limg/', str(timestamp) + '_axis.png'), im_with_charuco_board)
        print("Finish processing: {}".format(timestamp))
        R, _ = cv2.Rodrigues(rvecs)
        return R, tvecs, imgpts[0]
    else:
        # os.remove(img_filepath)
        print("Cannot find charuco in {}".format(timestamp))
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
    imgpts_list = []
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
    # imgpts_list.append(imgpts_org)

    r = Rot.from_matrix(R)
    quat = r.as_quat()

    return t, quat


def extractPose(args, cam_mtx, cam_dist):
    bridge = CvBridge()
    count = 0
    csv_file = open(args.csv_file_name, 'w')

    start = time.time()
    img_sec_list_l, img_sec_list= [], []
    with rosbag.Bag(args.bag_file, 'r') as bag:
        if args.stereo:
            start = time.time()
            for topic, msg, t in bag.read_messages(topics = args.l_img_topic):
                img_sec = msg.header.stamp.to_sec()
                img_sec_list_l.append(img_sec)

                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
                except CvBridgeError as e:
                    print(e)

                t, quat = getChessPoses(cv_image, img_sec, cam_mtx, cam_dist, charuco=True, pattern=(8,11), width=12)
                csv_file.write(
                str(img_sec) + ', ' +
                str(t[0,0]) + ', ' + str(t[1,0]) + ', ' +
                str(t[2,0]) + ', ' + str(quat[0]) + ', ' +
                str(quat[1]) + ', ' + str(quat[2]) + ', ' +
                str(quat[3]) + '\n')

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
            return img_sec_list_l

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

    data_dir = args.output_dir +'/limg/'
    cam_mtx = np.zeros((3,3))
    cam_mtx[0, 0] = 100
    cam_mtx[1, 1] = 100
    cam_mtx[0, 2] = 270
    cam_mtx[1, 2] = 480
    # print(cam_mtx)
    cam_dist = np.zeros([5])
    extractPose(args, cam_mtx, cam_dist)



if __name__ == '__main__':
    main()
    # time_stamp = np.load('time_stamp.npy')
    # print(len(time_stamp))
