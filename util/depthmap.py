import os
import ros_numpy
import rospy
import numpy as np
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import cv2
import matplotlib.pyplot as plt
import open3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors

bridge = CvBridge()

class SqueezedNorm(matplotlib.colors.Normalize):
    def __init__(self, vmin=None, vmax=None, mid=0, s1=2, s2=2, clip=False):
        self.vmin = vmin # minimum value
        self.mid  = mid  # middle value
        self.vmax = vmax # maximum value
        self.s1=s1; self.s2=s2
        f = lambda x, zero,vmax,s: np.abs((x-zero)/(vmax-zero))**(1./s)*0.5
        self.g = lambda x, zero,vmin,vmax, s1,s2: f(x,zero,vmax,s1)*(x>=zero) - \
                                             f(x,zero,vmin,s2)*(x<zero)+0.5
        matplotlib.colors.Normalize.__init__(self, vmin, vmax, clip)

    def __call__(self, value, clip=None):
        r = self.g(value, self.mid,self.vmin,self.vmax, self.s1,self.s2)
        return np.ma.masked_array(r)


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
    # halve precision to save storage
    scaled_depth = scaled_depth.astype(np.float16)
    # reverse height direction due to AMBF reshaping
    scaled_depth = np.ascontiguousarray(scaled_depth.reshape([h, w, 3])[::-1])
    # convert to cv convention
    scaled_depth = np.einsum(
        'ab,hwb->hwa', extrinsic[:3, :3], scaled_depth)[..., -1]
    return scaled_depth

def rospc_to_o3dpc(rospc, remove_nans=False):
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
    
    cloud_npy[...,0] = cloud_array['x']* scale
    cloud_npy[...,1] = cloud_array['y']* scale
    cloud_npy[...,2] = cloud_array['z']* scale
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
    global depth_data, segm_data, extrinsic, scale, pcd

    scaled_depth = depth_gen(depth)
    depth_data.append(scaled_depth)

    segm_mask = image_gen(segm) 
    segm_data.append(segm_mask)

    pcd = rospc_to_o3dpc(depth, remove_nans=False)
    print(depth.header.stamp.secs)

def listener():
    global depth_data, segm_data, extrinsic, scale, pcd

    segm = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/ImageData/compressed', CompressedImage)
    depth = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/DepthData', PointCloud2)

    rospy.init_node('listener', anonymous=True)
    ts = message_filters.ApproximateTimeSynchronizer([depth, segm], 50, 0.5)
    ts.registerCallback(callback)
    
    rospy.spin()
    rospy.on_shutdown(my_shutdown_hook)

def loadDepthMap(depth_data):
    norm=SqueezedNorm(vmin=0, vmax=0.26, mid=0.1386, s1=1.79, s2=2.45)
    for i in range(len(depth_data)):
        # depth_data[i] = np.where(depth_data[i]>1, 1, depth_data[i])
        print(depth_data[i].shape)
        plt.imshow(depth_data[i], vmin=0, vmax=0.26, cmap='Blues', norm=norm) #twilight #binary
        plt.colorbar(label='depth m', orientation='vertical', shrink=0.8)
        plt.axis('off')
        plt.show()
        break
    open3d.io.write_point_cloud('exp_3.pcd', pcd)

if __name__ == '__main__':
    depth_data = []
    segm_data = []
    pcd = []
    scale = 0.180
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]]) 
    listener()
    print('\nfinished!', f'{len(depth_data)} frames of depth map.')
    loadDepthMap(depth_data)