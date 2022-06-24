import numpy as np
import pandas as pd
from util.dataLoader import dataLoader
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

ld = dataLoader()


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

    def regPanel(self, csv_data):
        '''
        Registration for panel
        '''
        fs = ld.frames(len(csv_data), csv_data, panel=True)
        fs = np.array(fs)
        fs_mean = fs.mean(0)
        centroid = fs_mean.mean(0)
        # print(fs_mean, centroid)
        ci = fs_mean - centroid
        # print(ci)
        F = self.solvePointCloudReg_Arun(ci, fs_mean)
        # print(F)
        return F

    def plotView(self, inputArray):
        '''
        plot out 3D scatter points in array.
        '''

        # temp = np.array([[-4.75018009e+00, 8.27906023e+01, 2.49995459e+00],
        #                  [-9.43352098e-01, 1.31873134e+01, 2.72177732e-02],
        #                  [-4.12360849e+01, -1.52450943e+01, -8.91917505e+00],
        #                  [4.35646745e+01, -1.86076771e+01, 8.16388892e+00],
        #                  [3.36494255e+00, -6.21251443e+01, -1.77188623e+00]])
        x = inputArray[:, 0]
        y = inputArray[:, 1]
        z = inputArray[:, 2]

        ax = plt.subplot(projection='3d')
        ax.set_title('3d_image_show')
        ax.scatter(x, y, z, c = 'r')
        # ax.scatter(x[0], y[0], z[0], c='r')
        # ax.scatter(x[1], y[1], z[1], c='b')
        # ax.scatter(x[2], y[2], z[2], c='y')
        # ax.scatter(x[3], y[3], z[3], c='g')
        # ax.scatter(x[4], y[4], z[4], c='w')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    def invTransformation(self, trans):
        '''

        '''
        R = trans[:3, :3]
        t = trans[:3, 3].reshape(3, 1)
        R_new = R.T
        t_new = -R.T@t
        inv_trans = np.vstack((np.hstack((R_new, t_new)), np.array([0,0,0,1])))
        return inv_trans

    def trackTip(self, data, t_tip=np.array([-12.48857839 ,241.80863525 , 7.7726727])):
        '''
        The geometry used is geometryTry2
        t_tip = [-12.25806537, 242.03838824, 7.23566882] 2022/2/3
        t_tip = [-13.51879799 241.58888601   6.89874671] 2022/2/6
        t_tip = [-12.48857839 241.80863525   7.7726727] 2022/2/8
        '''
        t_i = data[0:3]
        q_i = data[3:7]
        t_i_shaped = np.array(t_i.reshape(-1, 1))
        r_i = Rotation.from_quat(q_i)
        # print('t_tip', t_i_shaped.shape)
        t_tip = np.reshape(t_tip,(-1, 1))
        r = r_i.as_matrix()
        p = r@t_tip + t_i_shaped
        return p

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

