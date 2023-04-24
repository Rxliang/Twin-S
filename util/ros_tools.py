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
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from ctypes import * # convert float to uint32
import open3d
import numpy as np
import matplotlib as mpl
import cv2
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
        '''
        Read the tf from rosbag and write the poses into csv.
        '''

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

    def rospc_to_o3dpc(self, rospc, remove_nans=False):
        """ covert ros point cloud to open3d point cloud
        Args: 
            rospc (sensor.msg.PointCloud2): ros point cloud message
            remove_nans (bool): if true, ignore the NaN points
        Returns: 
            o3dpc (open3d.geometry.PointCloud): open3d point cloud
        """
        field_names = [field.name for field in rospc.fields]
        is_rgb = 'rgb' in field_names
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(rospc).ravel()
        if remove_nans:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
        if is_rgb:
            cloud_npy = np.zeros(cloud_array.shape + (4,), dtype=np.float)
        else: 
            cloud_npy = np.zeros(cloud_array.shape + (3,), dtype=np.float)
        
        cloud_npy[...,0] = cloud_array['x']
        cloud_npy[...,1] = cloud_array['y']
        cloud_npy[...,2] = cloud_array['z']
        o3dpc = open3d.geometry.PointCloud()

        if len(np.shape(cloud_npy)) == 3:
            cloud_npy = np.reshape(cloud_npy[:, :, :3], [-1, 3], 'F')
        o3dpc.points = open3d.utility.Vector3dVector(cloud_npy[:, :3])

        if is_rgb:
            rgb_npy = cloud_array['rgb']
            rgb_npy.dtype = np.uint32
            r = np.asarray((rgb_npy >> 16) & 255, dtype=np.uint8)
            g = np.asarray((rgb_npy >> 8) & 255, dtype=np.uint8)
            b = np.asarray(rgb_npy & 255, dtype=np.uint8)
            rgb_npy = np.asarray([r, g, b])
            rgb_npy = rgb_npy.astype(np.float)/255
            rgb_npy = np.swapaxes(rgb_npy, 0, 1)
            o3dpc.colors = open3d.utility.Vector3dVector(rgb_npy)
        return o3dpc


    def project_pointcloud_to_image(pointcloud, intrinsics, extrinsics, image_shape):
        # Convert point cloud from 3D coordinates to 2D image coordinates
        points_2d, _ = cv2.projectPoints(pointcloud, extrinsics[:3], extrinsics[3:], intrinsics, distCoeffs=None)
        # Scale the 2D points to the size of the image
        points_2d = np.squeeze(points_2d)
        points_2d[:, 0] = np.clip(points_2d[:, 0], 0, image_shape[1] - 1)
        points_2d[:, 1] = np.clip(points_2d[:, 1], 0, image_shape[0] - 1)
        points_2d = np.round(points_2d).astype(int)

        # Create an empty image
        image = np.zeros((image_shape[0], image_shape[1], 3), dtype=np.uint8)
        dists = np.linalg.norm(pointcloud[:, :3], axis=1)  # compute distances from point to camera

        # # Assign RGB colors to the projected points
        cmap_norm = mpl.colors.Normalize(vmin=np.min(dists), vmax=np.max(dists))
        pixs_colors = plt.get_cmap('jet')(cmap_norm(dists))[:, 0:3]*255
        for i, point in enumerate(points_2d):
            c_ = tuple(pixs_colors[i].astype(np.uint8))
            c_ = np.uint8(c_)
            x, y = point
            r, g, b = int(c_[0]),int(c_[1]),int(c_[2])
            image[y, x] = [r, g, b]
        return image


    def saveImagesFromBag(self, bag, topics, img_sec_list, path):
        count = 0
        for topic, msg, t in bag.read_messages(topics):
            img_sec = msg.header.stamp.to_sec()
            # img_sec_list.append(img_sec)

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


    def rosmsg2quat(self, msg):
            '''
            transfer a rosmsg pose to seven params.
            '''
            t_x = msg.pose.position.x 
            t_y = msg.pose.position.y 
            t_z = msg.pose.position.z 
            x = msg.pose.orientation.x
            y = msg.pose.orientation.y
            z = msg.pose.orientation.z
            w = msg.pose.orientation.w
            conv_quat = [t_x,t_y,t_z,x,y,z,w]
            return conv_quat