import cv2
from cv_bridge import CvBridge
import numpy as np
import rosbag
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
# import h5py
import time
import logging
import argparse
import os
# import bagpy
import matplotlib.pyplot as plt
import yaml
import time
from PIL import Image
# import ros_numpy

# output_dir = "./Data"
input_dir = "./"
IMG_W = 640
IMG_H = 180
DATA_NUM = 3000
bridge = CvBridge()

def verify_cv_bridge():
    arr = np.zeros([480, 640])
    msg = bridge.cv2_to_imgmsg(arr)
    try:
        bridge.imgmsg_to_cv2(msg)
    except ImportError:
        log.log(logging.WARNING, "libcv_bridge.so: cannot open shared object file. Please source ros env first.")
        return False

    return True

def verify_ROS_connection():
    os.system("ping -c 4 10.42.0.27")
#     os.system("ssh 10.42.0.27")
    #Under Construction
    #if 
    #Return True

def initialization():
    global file , args
    
    parser = argparse.ArgumentParser(description="Extract images/steering angle/speed from a time-synchronized rosbag to a PyTable hdf5 file.")
    # parser.add_argument("img_dir", help="Input image directory", default='./test.bag', type=str)
    parser.add_argument("bag_file", help="Input ROS bag directory", default='./test.bag', type=str)
    parser.add_argument("output_dir", help="Output directory.", default='./Data', type=str)
    args = parser.parse_args()
    valid = verify_cv_bridge()
    # verify_ROS_connection()
    limg_path = args.output_dir +'/limg/'
    rimg_path = args.output_dir +'/rimg/'
    if valid:
        if not os.path.exists(args.output_dir):
            os.makedirs(args.output_dir)
            print("Create Folder at Designated Address...")
        if not os.path.exists(limg_path):
            os.makedirs(limg_path)
            print("Create Folder at Designated Address limg...")
        if not os.path.exists(rimg_path):
            os.makedirs(rimg_path)
            print("Create Folder at Designated Address rimg...")
        time_str = time.strftime("%Y%m%d_%H%M%S")
        # file = h5py.File(args.output_dir + '/' + time_str + ".hdf5", "w")
        
        #Under Construction
    else:
        print("Failed")

def bag2rawHDF5(args):
    global file
    sec_list = []
    nsec_list = []
    limg_list =[]
    rimg_list = []
    pose_pan = []
    pose_drill = []

    bag = rosbag.Bag(args.bag_file, "r")
    # limage = bag.message_by_topic('/fwd_limage/compressed')
    # print(limage.header.stamp.secs)

    #Read to Initialize the size for hdf5
    info_dict = yaml.load(bag._get_yaml_info())
    # duration = info_dict['duration']
    # start_time = info_dict['start']
    # end_time = info_dict['end']
    # print(start_time,duration)
    # print(end_time)
    print(info_dict)
    size = info_dict["topics"][0]['messages']
    print("Number of Messages: ",size)
    # limg_dst = file.create_dataset('Pose_Panel',(size,1080,1920,3),dtype = "float64" ,chunks = True)
    # rimg_np = np.empty([size,1080,1920,3])
    # rimg_np = np.memmap('hard_disk_rimg', dtype=np.float64, mode='w+',shape=(size,1080,1920,3))
    # print limg_dst.dtype
    count= 0
    start = time.time()
    img_sec_list_r, img_sec_list_l = [], []
    for topic, msg, t in bag.read_messages(topics = "/fwd_rimage/compressed"):
    # for topic, msg, t in bag.read_messages(topics = "/decklink_right/camera/image_raw/compressed"):
    #     # print(msg.__slots__)
    #     # break
        # if topic == '/fwd_limage/compressed':
        img_sec = msg.header.stamp.nsecs
        img_sec_list_r.append(img_sec)
        # img_nsec = msg.header.stamp.nsecs

        #     # sec_list.append(img_sec)
            # nsec_list.append(img_nsec)
        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        file_path = args.output_dir +'/rimg/' + str(count) + '.jpeg'
        pil_image = Image.fromarray(cv_img[:,:,::-1])
        pil_image.save(file_path)
        count+=1
        print(count)
            # pose_pan.append(msg)
        # if topic == '/fwd_pose_drill':
            # pose_drill.append(msg)
    count = 0
    for topic, msg, t in bag.read_messages(topics = "/fwd_limage/compressed"):
    # for topic, msg, t in bag.read_messages(topics = "/decklink_left/camera/image_raw/compressed"):
        img_sec = msg.header.stamp.nsecs
        img_sec_list_l.append(img_sec)
        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # rimg_np[count] = cv_img
        file_path = args.output_dir +'/limg/' + str(count) + '.jpeg'
        pil_image = Image.fromarray(cv_img)
        pil_image.save(file_path)
        count+=1
        print(count)

    end = time.time()
    print(end - start)
    print("Complete Saving")
    return img_sec_list_l, img_sec_list_r
    # print(len(sec_list))
    # file["sec"] = sec_list    

    
def test_memmap():
    rimg_np = np.memmap('hard_disk_rimg', dtype=np.float64, mode='w+',shape=(1080,1920,3))
    rimg_np =  np.zeros([1080,1920,3])
    np.save("test.npy",rimg_np)

def test_sync(l_ref,r_test):
     l_ref, r_test = np.asarray(l_ref), np.asarray(r_test)
     np.save("l_ref",l_ref)
     np.save("r_test",r_test)
     print(l_ref[-1]-l_ref[0])
     print(l_ref[-1],r_test[-1])
     print(r_test[-1]-r_test[0])
# def verify_saved():
#     np.load("rimg_cal.npy")
# def sample_memmap(file_name):
    

def main():
    global args
    
    initialization()
    l,r = bag2rawHDF5(args)
    # print(l[283])
    # test_sync(l,r)
    # test_memmap()
if __name__ =="__main__":
    main()