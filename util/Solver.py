import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import cv2
import os
from natsort import natsorted
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
# ld = dataLoader()


class solver:

    def solvePointCloudReg_Arun(self, a, b):
        '''Method due to K.Arun, et. al., IEEE PAMI, Vol 9, no 5, pp 698-700, Sept 1987
            @param a: input 3nx3 np.mat b = Fa
            @param b: input 3nx3 np.mat b = Fa
            @return: R, p components of F
        '''
        a_mean = a.mean(0)
        # print(a_mean)
        b_mean = b.mean(0)

        a_hat = np.mat(a - a_mean)
        b_hat = np.mat(b - b_mean)
        # print(a_hat[1].T * b_hat[1])
        # compute H
        H = []
        for i in range(len(a)):
            H.append(a_hat[i].T * b_hat[i])
        H = np.vstack(sum(H))
        # print(H)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T * U.T
        # print(U,'\n', Vt,'\n', H)
        det = np.linalg.det(R)
        # print(R)
        if det < 0:
            V = Vt.T * [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
            R = V * U.T
        p = np.mat(b_mean).T - R * np.mat(a_mean).T
        F = [R, p]
        return F


    def pivotCalibration(self, frames):
        '''
        @param num_frame: number of frames
        @param markers: numpy array of markers coordinates [m1; m2; m3; m4; m5]
        '''
        R = []
        p = []
        I = []
        num_frame = len(frames)
        # get mean point of frame 1 markers
        G_0 = frames[0].mean(0)
        for j in range(len(frames[0])):
            gi = frames[0] - G_0
        for i in range(num_frame):
            Gi = frames[i]
            # print(gi)
            # print(Gi)
            F = self.solvePointCloudReg_Arun(gi, Gi)
            R.append(F[0])
            p.append(F[1])
            I.append(np.identity(3))
        R = np.vstack(R)
        p = np.vstack(p)
        I = np.vstack(I)

        #set a lstsq problem
        X = np.hstack((I, -R))
        # print(R)
        w = np.linalg.lstsq(X, p, None)[0]
        p_dimple = w[:3]
        t_tip = w[3:]
        # print(p_dimple)
        return p_dimple, t_tip


    def seven2trans(self, data):
        '''
        Seven parameter to transformation
        '''
        t_i = data[0:3]
        q_i = data[3:7]
        t_i_shaped = np.array(t_i.reshape(-1, 1))
        r_i = Rotation.from_quat(q_i)
        r = r_i.as_matrix()
        trans = np.vstack((np.hstack((r, t_i_shaped)), np.array([0, 0, 0, 1])))
        return [r, t_i_shaped], trans


    def trans2seven(self, data):
        '''
        transformation to Seven parameter
        '''
        R = data[:3, :3]
        r_i = Rotation.from_matrix(R)
        q = r_i.as_quat()
        t = data[:3, 3]
        return np.array([t[0],t[1],t[2],q[0],q[1],q[2],q[3]])


    def visulizePoints(self, point_array):
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


    def invTransformation(self, trans):
        '''
        Inverse of the SE(3) transformation.
        '''
        R = trans[:3, :3]
        t = trans[:3, 3].reshape(3, 1)
        R_new = R.T
        t_new = -R.T@t
        inv_trans = np.vstack((np.hstack((R_new, t_new)), np.array([0,0,0,1])))
        return inv_trans


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


    def get_mat_log(self, R):
        """
        Get the log(R) of the rotation matrix R.

        Args:
        - R (3x3 numpy array): rotation matrix
        Returns:
        - w (3, numpy array): log(R)
        """
        theta = np.arccos((np.trace(R) - 1) / 2)
        w_hat = (R - R.T) * theta / (2 * np.sin(theta))  # Skew symmetric matrix
        w = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])  # [w1, w2, w3]
        return w


    def img2video(self, loadPath, saveName, fps, size):
         # cv record video
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        saveName = os.path.join(loadPath, saveName)
        videoWriter = cv2.VideoWriter(saveName, fourcc, fps, size)
        img_list = os.listdir(loadPath)
        img_list = natsorted(img_list)
        count = 0
        for file in img_list:
            img = cv2.imread(os.path.join(loadPath, file))
            videoWriter.write(img)
            count += 1
            print(f'finished {count}.')


    def make_charuco_pattern(self, path):
        charuco_pattern = np.zeros([88, 3])
        count = 0
        for i in range(11):
            for j in range(8):
                charuco_pattern[count] = np.array([i*15,j*15,0])
                count += 1
        # print(charuco_pattern)
        charuco_pattern = np.save(path, charuco_pattern)


    def csv2txt(self, src, dest):

        with open(src) as file:
            output = ""
            line = file.readline()
            while line != '':
                line_values = line.split(',')
                output += line_values[0] + ''
                for i in range(7):
                    output += line_values[i+1] + ""
                # output += '\n'
                line = file.readline()

        with open(dest, mode='w') as file:
            file.writelines(output)


    def crop_Pointcloud(self, pcd_path):
        '''
        Crop the pointcloud with open3d.
        '''
        print("Load a ply point cloud, print it, and render it")
        # Crop the pointcloud
        # pcd = o3d.io.read_point_cloud('../data/phantom_point-cloud_data/phacon_exp_4.ply')
        pcd = o3d.io.read_point_cloud(pcd_path)

        print("Demo for manual geometry cropping")
        print(
            "1) Press 'Y' twice to align geometry with negative direction of y-axis"
        )
        print("2) Press 'K' to lock screen and to switch to selection mode")
        print("3) Drag for rectangle selection,")
        print("   or use ctrl + left click for polygon selection")
        print("4) Press 'C' to get a selected geometry")
        print("5) Press 'S' to save the selected geometry")
        print("6) Press 'F' to switch to freeview mode")

        o3d.visualization.draw_geometries_with_editing([pcd])