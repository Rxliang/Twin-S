from glob import glob
import rosbag
import rospy
import ros_numpy
import bagpy
import cv2
import numpy as np
import os
import time
import matplotlib.pyplot as plt
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
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
    global file , args, path
    
    parser = argparse.ArgumentParser(description="Extract images from rosbag.")
    parser.add_argument('--bag',dest="bag_file", help="Input ROS bag directory", default='./test.bag', type=str)
    parser.add_argument('--outdir',dest="output_dir", help="Output directory.", default='./Data', type=str)
    parser.add_argument('--stereo',dest="stereo", help="Stereo or monocular.", action='store_true')
    parser.add_argument('--limg',dest="limg", help="Left image only.", action='store_true')
    parser.add_argument('--panel',dest="pan", help="With phacon panel or not.", action='store_true')
    parser.add_argument('--l_topic',dest="l_img_topic", help="Topic of left image.", default='/sync_limage/compressed', type=str)
    parser.add_argument('--r_topic',dest="r_img_topic", help="Topic of right image.", default='/fwd_rimage/compressed', type=str)
    parser.add_argument('--segm',dest="segm", help="Segmentation masks.", action='store_true')
    parser.add_argument('--depth',dest="depth", help="Depth map.", action='store_true')
    parser.add_argument('--segm_topic',dest="segm_topic", help="Topic of segmentation.", default='/sync_segm/compressed', type=str)
    parser.add_argument('--depth_topic',dest="depth_topic", help="Topic of depth.", default='/sync_depthData', type=str)
    parser.add_argument('--pose',dest="pose", help="Poses of optical tracking.", action='store_true')
    # segm_topic = '/fwd_segm/compressed'
    # depth_topic = '/fwd_depthData'

    args = parser.parse_args()
    args.output_dir = args.bag_file[:-4]

    valid = verify_cv_bridge()
    # verify_ROS_connection()
    limg_path = args.output_dir +'/limg/'
    rimg_path = args.output_dir +'/rimg/'
    segm_path = args.output_dir +'/segm_mask/'
    depth_path = args.output_dir +'/depth_map/'
    path = [limg_path, rimg_path, segm_path, depth_path]

    if valid:
        if not os.path.exists(args.output_dir):
                os.makedirs(args.output_dir)
                print("Create Folder at Designated Address...")
        if args.stereo:
            if not os.path.exists(limg_path):
                os.makedirs(limg_path)
                print("Create Folder at Designated Address limg...")
            if not os.path.exists(rimg_path):
                os.makedirs(rimg_path)
                print("Create Folder at Designated Address rimg...")
            time_str = time.strftime("%Y%m%d_%H%M%S")
        elif args.limg:
            if not os.path.exists(limg_path):
                os.makedirs(limg_path)
                print("Create Folder at Designated Address limg...")
        if args.segm:
            if not os.path.exists(segm_path):
                os.makedirs(segm_path)
                print("Create Folder at Designated Address segm_mask...")
        if args.depth:
            if not os.path.exists(depth_path):
                os.makedirs(depth_path)
                print("Create Folder at Designated Address depth_path...")
    else:
        print("Failed")

def bag2csv(args):
    b = bagpy.bagreader(args.bag_file)
    if args.pan:
        b.message_by_topic('/fwd_pose_pan')
        b.message_by_topic('/fwd_pose_drill')
        b.message_by_topic('/fwd_pose_camhand')
    else:
        b.message_by_topic('/fwd_pose_drill')
        b.message_by_topic('/fwd_pose_camhand')
    print("Complete Saving csv.")

def saveImagesFromBag(bag, topics, img_sec_list, path):
    global count
    for topic, msg, t in bag.read_messages(topics):
        img_sec = msg.header.stamp.to_sec()
        img_sec_list.append(img_sec)

        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        image_path = path + str(count) + '.jpeg'
        pil_image = Image.fromarray(cv_image[:,:,::-1])
        pil_image.save(image_path)
        count += 1
        print(path +' '+ str(count))


def saveDepthImagesFromBag(bag, topics, path):
    global count
    
    scale = 0.194
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]]) 
    [h, w] = [480, 640]

    for topic, depth_msg, t in bag.read_messages(topics):
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)
        xcol = xyz_array['x'][:, None] * scale
        ycol = xyz_array['y'][:, None] * scale
        zcol = xyz_array['z'][:, None] * scale

        scaled_depth = np.concatenate([xcol, ycol, zcol], axis=-1)
        # halve precision to save storage
        scaled_depth = scaled_depth.astype(np.float16)
        # reverse height direction due to AMBF reshaping
        scaled_depth = np.ascontiguousarray(scaled_depth.reshape([h, w, 3])[::-1])
        # convert to cv convention
        scaled_depth = np.einsum(
            'ab,hwb->hwa', extrinsic[:3, :3], scaled_depth)[..., -1]
        # visualize
        image_path = path + str(count) + '.png'
        plt.imshow(scaled_depth, vmin=0, vmax=0.26, cmap='Blues') #twilight #binary
        plt.colorbar(label='depth m', orientation='vertical')
        plt.savefig(image_path)
        plt.close()
        count += 1
        print(path +' '+ str(count))


def bag2images(args):
    global count, path
    bridge = CvBridge()
    count = 0
    start = time.time()
    img_sec_list_l, img_sec_list= [], []
    

    with rosbag.Bag(args.bag_file, 'r') as bag:
        if args.stereo:
            start = time.time()
            saveImagesFromBag(bag, args.l_img_topic, img_sec_list_l, path[0])

            count = 0
            saveImagesFromBag(bag, args.r_img_topic, img_sec_list, path[1])

        elif args.limg:
            saveImagesFromBag(bag, args.l_img_topic, img_sec_list, path[0])

        if args.segm:
            count = 0
            saveImagesFromBag(bag, args.segm_topic, img_sec_list, path[2])

        if args.depth:
            count = 0
            saveDepthImagesFromBag(bag, args.depth_topic, path[3])
        end = time.time()
        print(end - start)
        print("Complete Saving")
        return img_sec_list

def main():
    global args
    initialization()
    if args.pose:
        bag2csv(args)
    img_sec_list = bag2images(args)

    # np.save('time_stamp',img_sec_list)

    # data_dir = args.output_dir +'/img/'
    # cam_mtx = np.zeros((4,4))
    # cam_dist = np.zeros([5])

    # marker_poses, imgpts_list = sp.getChessPoses(data_dir, cam_mtx, cam_dist, charuco=False, pattern=(6,8), width=20)




if __name__ == '__main__':
    main()
    # print(len(time_stamp))
