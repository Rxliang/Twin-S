#!/usr/bin/env python
import argparse
import math
import sys
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rosbag
import rospy
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, Image
from PIL import Image
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from ctypes import * # convert float to uint32
import open3d
bridge = CvBridge()


class rostools:
    
    def __init__(self) -> None:
        pass
    
    def verify_cv_bridge(self):
        arr = np.zeros([480, 640])
        msg = bridge.cv2_to_imgmsg(arr)
        try:
            bridge.imgmsg_to_cv2(msg)
        except ImportError:
            # log.log(logging.WARNING, "libcv_bridge.so: cannot open shared object file. Please source ros env first.")
            return False
        return True


    def write_transformation_to_csv_file(self, bag_file, target_frame,
                                     csv_file_name):
        print("Loading tfs into Transformer...")
        bag = rosbag.Bag(bag_file)

        print("Listening to tf transformation:", target_frame)
        # Reopen bag
        bag = rosbag.Bag(bag_file)
        init = True
        tf_counter = 0
        tf_frequency_estimated = 0.
        start_time_tf_message = rospy.Time()
        end_time_tf_message = rospy.Time()
        csv_file = open(csv_file_name, 'w')
        print("Looking up transforms and writing to csv file...")
        for topic, msg, t in bag.read_messages():

                # Initialize start time to first successful tf message lookup.
                if topic == target_frame:
                    if init:
                        start_time_tf_message = msg.header.stamp
                        init = False

                    # Write to csv file.
                    csv_file.write(
                        str(msg.header.stamp.to_sec()) + ', ' +
                        str(msg.pose.position.x) + ', ' + str(msg.pose.position.y) + ', ' +
                        str(msg.pose.position.z) + ', ' + str(msg.pose.orientation.x) + ', ' +
                        str(msg.pose.orientation.y) + ', ' + str(msg.pose.orientation.z) + ', ' +
                        str(msg.pose.orientation.w) + '\n')

                    # # Update end time.
                    end_time_tf_message = msg.header.stamp
                    tf_counter += 1

        # Output final estimated frequency.
        if tf_counter > 3:
            tf_frequency_estimated = tf_counter / (
                end_time_tf_message - start_time_tf_message).to_sec()
            print("Final estimate of tf topic frequency: ", "{0:.2f}".format(
                tf_frequency_estimated), "Hz")

        print("Exported ", tf_counter, " tf poses.")

        
    def convertCloudFromOpen3dToRos(self, open3d_cloud, frame_id="odom"):
        '''
        Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
        '''

        # The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        FIELDS_XYZRGB = FIELDS_XYZ + \
            [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

        # Bit operations
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.points)
        if not open3d_cloud.colors: # XYZ only
            fields=FIELDS_XYZ
            cloud_data=points
        else: # XYZ + RGB
            fields=FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.
            colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
            colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
            cloud_data=np.c_[points, colors]
        
        # create ros_cloud
        return pc2.create_cloud(header, fields, cloud_data)


    def convertCloudFromRosToOpen3d(self, ros_cloud):

        convert_rgbUint32_to_tuple = lambda rgb_uint32: (
        (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
        )
        convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
            int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
        )
        
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            scale = 1000
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
            # print(open3d_cloud.points[200000])
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    def saveImagesFromBag(self, bag, topics, img_sec_list, path):
        count = 0
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
            
            print(path +' '+ str(count))
            count += 1


    def saveDepthImagesFromBag(bag, topics, scale, path):
        '''
        Save Pointcloud2 to depth map from AMBF. 
        '''
        count = 0
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

            # # visualize
            # image_path = path + str(count) + '.png'
            # norm=SqueezedNorm(vmin=0, vmax=0.26, mid=0.13, s1=1.79, s2=2.45)
            # plt.imshow(scaled_depth, vmin=0, vmax=0.26, cmap='Blues', norm=norm) #twilight #binary
            # plt.colorbar(label='depth m', orientation='vertical', shrink=0.8)
            # plt.axis('off')
            # plt.savefig(image_path)
            # plt.close()
            
            print(path +' '+ str(count))
            # np.save(os.path.join(path, str(count)), scaled_depth)
            count += 1