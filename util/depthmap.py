import os
import ros_numpy
import rospy
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, PointCloud2
from geometry_msgs.msg import PoseStamped
import cv2
import matplotlib.pyplot as plt
import open3d as o3d
from mpl_toolkits.mplot3d import Axes3D

bridge = CvBridge()

def visulizePoints(point_array):
    x = point_array[:, 0]
    y = point_array[:, 1]
    z = point_array[:, 2]

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z)
    
    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()

def depth_gen(depth_msg):
    """
    generate depth
    :param depth_msg:
    :return: HxW, z-values
    """
    global pointcloud
    [h, w] = [480, 640]
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)
    xcol = xyz_array['x'][:, None] * scale
    ycol = xyz_array['y'][:, None] * scale
    zcol = xyz_array['z'][:, None] * scale

    scaled_depth = np.concatenate([xcol, ycol, zcol], axis=-1)
    pointcloud = scaled_depth
    # halve precision to save storage
    scaled_depth = scaled_depth.astype(np.float16)
    # reverse height direction due to AMBF reshaping
    scaled_depth = np.ascontiguousarray(scaled_depth.reshape([h, w, 3])[::-1])
    # convert to cv convention
    scaled_depth = np.einsum(
        'ab,hwb->hwa', extrinsic[:3, :3], scaled_depth)[..., -1]
    return scaled_depth

def image_gen(image_msg):
    try:
        cv2_img = bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        return cv2_img
    except CvBridgeError as e:
        print(e)
        return None

def my_shutdown_hook():
    print("in my_shutdown_hook")

def callback(depth, segm):
    global depth_data, segm_data, extrinsic, scale, pointcloud

    scaled_depth = depth_gen(depth)
    depth_data.append(scaled_depth)

    segm_mask = image_gen(segm) 
    segm_data.append(segm_mask)

    print(depth.header.stamp.secs)

def listener():
    global depth_data, segm_data, extrinsic, scale, pointcloud

    segm = message_filters.Subscriber('fwd_segm/compressed', CompressedImage)
    depth = message_filters.Subscriber('fwd_depthData', PointCloud2)

    rospy.init_node('listener', anonymous=True)
    ts = message_filters.ApproximateTimeSynchronizer([depth, segm], 50, 0.5)
    ts.registerCallback(callback)
    
    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

def loadDepthMap(depth_data):
    for i in range(len(depth_data)):
        # depth_data[i] = np.where(depth_data[i]>1, 1, depth_data[i])
        print(depth_data[i].shape)
        plt.imshow(depth_data[i], vmin=0, vmax=0.26, cmap='Blues') #twilight #binary
        plt.colorbar(label='depth m', orientation='vertical')
        plt.show()
        break

if __name__ == '__main__':
    depth_data = []
    segm_data = []
    scale = 0.194
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]]) 
    pointcloud = []
    listener()
    print('\nfinished!', f'{len(depth_data)} frames of depth map.')
    loadDepthMap(depth_data)