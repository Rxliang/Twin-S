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
from ros_tools import rostools
from Solver import solver
bridge = CvBridge()
rt = rostools()
sol = solver()

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


def depth_gen(depth_msg):
    """
    generate depth
    :param depth_msg:
    :return: HxW, z-values
    """
    global pointcloud
    # [h, w] = [480, 640]
    [h, w] = [360, 640]
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

    # scaled_depth = depth_gen(depth)
    scaled_depth = rt.rospc_to_o3dpc(depth)
    # depth_data.append(scaled_depth)

    segm_mask = image_gen(segm) 
    segm_data.append(segm_mask)

    pcd = rt.rospc_to_o3dpc(depth)
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
        plt.imshow(depth_data[i], vmin=0.1, vmax=0.26, cmap='Spectral_r', norm=norm) #twilight #binary
        plt.colorbar(label='depth m', orientation='vertical', shrink=0.8)
        plt.axis('off')
        plt.show()
        break
    # open3d.io.write_point_cloud('eeeeeeeeeeeeeeeexp.pcd', pcd)
    
if __name__ == '__main__':
    depth_data = []
    segm_data = []
    pcd = []
    scale = 0.180
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]]) 
    listener()
    print('\nfinished!', f'{len(depth_data)} frames of depth map.')
    # loadDepthMap(depth_data)
    # sol.crop_Pointcloud('cropped_exp2.ply')