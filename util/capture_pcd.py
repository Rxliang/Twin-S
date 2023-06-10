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
from pynput import keyboard
import json
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

def quat2trans(quat):
    """
    Quaternion to 4x4 transformation.
    
    Args:
    - quat (4, numpy array): quaternion w, x, y, z
    Returns:
    - rotm: (4x4 numpy array): transformation matrix
    """
    x = quat[3]
    y = quat[4]
    z = quat[5]
    w = quat[6]

    t_x = quat[0]
    t_y = quat[1]
    t_z = quat[2]

    s = w*w + x*x + y*y + z*z

    homo = np.array([[1-2*(y*y+z*z)/s, 2*(x*y-z*w)/s,   2*(x*z+y*w)/s  ,t_x],
                     [2*(x*y+z*w)/s,   1-2*(x*x+z*z)/s, 2*(y*z-x*w)/s  ,t_y],
                     [2*(x*z-y*w)/s,   2*(y*z+x*w)/s,   1-2*(x*x+y*y)/s, t_z],
                     [0,0,0,1]
    ])

    return homo

def pose_gen(pose_msg):
    pose = pose_msg.pose
    pose_np = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])

    return pose_np

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
    xcol = xyz_array['x'][:, None] / scale
    ycol = xyz_array['y'][:, None] / scale
    zcol = xyz_array['z'][:, None] / scale
    
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

# def callback(depth, segm):
#     global depth_data, segm_data, extrinsic, scale, pcd, T_o_db

#     # scaled_depth = depth_gen(depth)
#     scaled_depth = rt.rospc_to_o3dpc(depth)
#     # depth_data.append(scaled_depth)

#     segm_mask = image_gen(segm) 
#     segm_data.append(segm_mask)

#     pcd = rt.rospc_to_o3dpc(depth)    
#     print('*********************Timestamp: ', depth.header.stamp.secs, end='\r')

def callback(pose_drill, pose_camhand):
    global extrinsic, scale, pcd, T_o_db, T_c_o
    X = np.load('../params/hand_eye_X_optimized_0411.npy')
    X[:3, 3] /= 1000
    T_o_cb = quat2trans(sol.rosmsg2quat(pose_camhand))
    T_o_c = T_o_cb @ X
    T_c_o = sol.invTransformation(T_o_c)
    T_o_db = quat2trans(sol.rosmsg2quat(pose_drill))
    print('*********************Timestamp: ', pose_drill.header.stamp.secs, end='\r')

def listener():
    global depth_data, segm_data, extrinsic, scale, pcd, T_o_db

    segm = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/ImageData/compressed', CompressedImage)
    depth = message_filters.Subscriber('/ambf/env/cameras/segmentation_camera/DepthData', PointCloud2)
    pose_drill = message_filters.Subscriber('/fwd_pose_drill', PoseStamped)
    pose_camhand = message_filters.Subscriber('/fwd_pose_camhand', PoseStamped)

    rospy.init_node('listener', anonymous=True)
    # ts = message_filters.ApproximateTimeSynchronizer([depth, segm], 50, 0.5)
    ts = message_filters.ApproximateTimeSynchronizer([pose_drill, pose_camhand], 50, 0.5)

    ts.registerCallback(callback)
    # print('Press s to save point cloud from AMBF...')
    listener_k = keyboard.Listener(on_press=on_press)
    listener_k.start()
    listener_k.join()

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

def save_pcd():
    global count
    open3d.io.write_point_cloud(f'{count}.pcd', pcd)
    count += 1
    print("Point cloud saved!")

def save_tip():
    np.save('tip_pose.npy', tip_pose)
    print("Tip pose saved!")

def on_press(key):
    if key == keyboard.Key.esc:  # Check for the desired key, in this case 'esc'
        print('esc')
        return False  # Stop the listener
    try:
        if key.char == 's':  # Check for the desired key, in this case 's'
            # save_pcd()
            save_tip()

        if key.char == 'r':  # Check for the desired key, in this case 's'
            # save the drill tip location
            global T_db_d, tip_pose, count
            t_c_tip = (T_c_o @ T_o_db @ T_db_d)[:3, 3]
            tip_pose.append(t_c_tip)
            print(f"Added {count+1} pose!")
            count += 1

    except AttributeError:
        pass

if __name__ == '__main__':
    # Load the calibration parameters
    f = open('../conig/calibration_param_config.json')
    calib_config_path = json.load(f)
    ## Drill tip in drill coordinate t_tip from pivot calibration
    t_db_d = np.load(calib_config_path['root_path']+calib_config_path['t_db_d'])
    ## Rotation of tip w.r.t drill marker got from rotation calibration
    R_db_d =np.load(calib_config_path['root_path']+calib_config_path['R_db_d'])
    T_db_d = np.vstack([np.hstack([R_db_d, t_db_d/1000]),np.array([0,0,0,1]).reshape(1,4)])
    tip_pose = []
    depth_data = []
    segm_data = []
    pcd = []
    count = 0
    scale = 0.180
    extrinsic = np.array([[0, 1, 0, 0], [0, 0, -1, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]]) 
    listener()
    
    # loadDepthMap(depth_data)
    # sol.crop_Pointcloud('cropped_exp2.ply')