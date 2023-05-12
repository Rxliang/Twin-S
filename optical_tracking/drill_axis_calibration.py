import sys
sys.path.insert(0, '/home/shc/Twin-S/util')
from dataLoader import dataLoader
from Solver import solver

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as Rot
from sklearn.linear_model import RANSACRegressor

sol = solver()
ld = dataLoader()

def plotCoordinateFrame(axis, T_0f, size=1, linewidth=3):
    """Plot a coordinate frame on a 3d axis. In the resulting plot,
    x = red, y = green, z = blue.
    
    plotCoordinateFrame(axis, T_0f, size=1, linewidth=3)
    Arguments:
    axis: an axis of type matplotlib.axes.Axes3D
    T_0f: The 4x4 transformation matrix that takes points from the frame of interest, to the plotting frame
    size: the length of each line in the coordinate frame
    linewidth: the width of each line in the coordinate frame
    Usage is a bit irritating:
    import mpl_toolkits.mplot3d.axes3d as p3
    import pylab as pl
    f1 = pl.figure(1)
    # old syntax
    # a3d = p3.Axes3D(f1)
    # new syntax
    a3d = f1.add_subplot(111, projection='3d')
    # ... Fill in T_0f, the 4x4 transformation matrix
    plotCoordinateFrame(a3d, T_0f)
    see http://matplotlib.sourceforge.net/mpl_toolkits/mplot3d/tutorial.html for more details
    """
    # \todo fix this check.
    #if type(axis) != axes.Axes3D:
    #    raise TypeError("axis argument is the wrong type. Expected matplotlib.axes.Axes3D, got %s" % (type(axis)))

    # p_f = np.array([ [0,0,0,1], [size,0,0,1], [0,size,0,1], [0,0,size,1]]).T
    # p_0 = np.dot(T_0f,p_f)
    R = T_0f[:3, :3]
    t = T_0f[:3, 3].reshape(3, -1)
    # p_0 = T_0f
    # X-axis
    length = 10

    xline = np.array([np.linspace(0, size, length),np.zeros(length),np.zeros(length)]).T
    yline = np.array([np.zeros(length),np.linspace(0, size, length),np.zeros(length)]).T
    zline = np.array([np.zeros(length),np.zeros(length),np.linspace(0, size, length)]).T
    xline = R@xline.T+t 
    yline = R@yline.T+t
    zline = R@zline.T+t

    axis.plot3D(xline[0,:],xline[1,:], xline[2,:], 'r-', linewidth=linewidth)
    axis.plot3D(yline[0,:],yline[1,:], yline[2,:], 'g-', linewidth=linewidth)
    axis.plot3D(zline[0,:],zline[1,:], zline[2,:], 'b-', linewidth=linewidth)
    # axis.plot3D(X[:,0],X[:,1],X[:,2],'r-', linewidth=linewidth)
    # axis.plot3D(Y[:,0],Y[:,1],Y[:,2],'g-', linewidth=linewidth)
    # axis.plot3D(Z[:,0],Z[:,1],Z[:,2],'b-', linewidth=linewidth)
    
def getNdarray(pose_path):
    csv_data = pd.read_csv(pose_path)
    # print(csv_data.head())
    num_frames = len(csv_data)
    translation_array = []
    F_array = []
    pivot = np.load('/home/shc/Twin-S/params/drill_pivot_0425.npy')
    for i in range(num_frames):
        seven_params = ld.getToolPose(i, pose_path)
        _, F = sol.seven2trans(seven_params)
        # translation_array.append((F[:3, :3]@pivot).T + F[:3, 3])
        F_array.append(F)
        translation_array.append(F[:3, 3])
    translation_array = np.vstack(translation_array)
    F_array = np.asarray(F_array)
    return translation_array, F_array

def getNdarrayInv(pose_path):
    csv_data = pd.read_csv(pose_path)
    # print(csv_data.head())
    num_frames = len(csv_data)
    translation_array = []
    F_array = []
    pivot = np.load('/home/shc/Twin-S/params/drill_pivot_0425.npy')
    for i in range(num_frames):
        seven_params = ld.getToolPose(i, pose_path)
        _, F = sol.seven2trans(seven_params)
        # F_db_o
        F = sol.invTransformation(F)
        F_array.append(F)
        translation_array.append(F[:3, 3])
    translation_array = np.vstack(translation_array)
    F_array = np.asarray(F_array)
    return translation_array, F_array

def getNorm(translation_array):
    num_frames = len(translation_array)
    dislist = []
    for i in range(num_frames):
        try:
            dis = np.linalg.norm(translation_array[i+1]-translation_array[i])
            dislist.append(dis)
        except:
            dislist = np.vstack(dislist)
            return dislist

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

def visulizeFrames(F_array):
    fig = plt.figure()
    ax = Axes3D(fig)
    for F in F_array:
        plotCoordinateFrame(ax, F)
    plt.show()

def fit_plane(points, plane_coeff=None, inlier_thresh=0.00005, max_iterations=10000, RANSAC=False):
    """
    Fit a plane to a set of 3D points using the SVD method.
    Input: points (numpy array) - Nx3 array of 3D points
    Output: p (numpy array) - 1x4 array representing the plane coefficients
            normal (numpy array) - 1x3 array representing the normal vector of the plane
    """
    if RANSAC:
        model = RANSACRegressor(estimator=None, min_samples=30, residual_threshold=inlier_thresh,
                                max_trials=max_iterations, random_state=None)
        p_points = np.hstack([points, np.ones([points.shape[0], 1])])
        y = p_points@plane_coeff
        model.fit(points, y)
        inliers = model.inlier_mask_
        print('Total points:', points.shape[0], 'Inliers:', sum(inliers), 'Trails:', model.n_trials_)
        points = points[inliers]
    # Compute the centroid of the points
    centroid = np.mean(points, axis=0)
    
    # Subtract the centroid from the points
    points_centered = points - centroid
    
    # Use SVD to find the normal vector of the plane
    u, s, vh = np.linalg.svd(points_centered)
    normal = vh[2]
    
    # Compute the plane coefficients (Ax + By + Cz + D = 0)
    A, B, C = normal
    D = -np.dot(normal, centroid)
    plane_coeff = np.array([[A],[B],[C],[D]])
    normal = normal.reshape(1,3)
    # print(normal)
    # Return the plane coefficients and normal vector
    if RANSAC:
        return normal, plane_coeff, inliers
    else:
        return normal, plane_coeff

def axisCalibration_Kabsch(pose_path, translation_array):
    dislist = getNorm(translation_array)
    pointsInDrill = np.zeros([len(translation_array), 3])
    
    for i in range(len(dislist)):
        pointsInDrill[i+1, 0] = np.sum(dislist[:i+1])
    # return
    # scipy vector align to get the rotation
    (r, rmsd, sensi) = Rot.align_vectors(pointsInDrill, translation_array, return_sensitivity=True)
    R_o_tip = r.as_matrix()
    # R = r.as_euler('zyx', degrees=True)

    csv_data = pd.read_csv(pose_path)
    num_frames = len(csv_data)
    translation_array = []
    for i in range(num_frames):
        # quaternion = ld.getToolPose(i, csv_data)
        seven_params = ld.getToolPose(i, pose_path)
        _, F_o_drill = sol.seven2trans(seven_params)
        R_d_tip = sol.invTransformation(F_o_drill)[:3, :3]@R_o_tip
        r = Rot.from_matrix(R_d_tip)
        euler = r.as_euler('zyx', degrees=True)
        # print(R_d_tip)
    # print(R_d_tip)
    print('R:',R_d_tip)
    # np.save('../params/R_db_d.npy', R_d_tip)


def axisCalibration_RANSAC(translation_array, F_array):
    normal_db = []
    normal, plane_coeff = fit_plane(translation_array)
    normal, _, inliers = fit_plane(translation_array, plane_coeff=plane_coeff, RANSAC=True)

    drillAxis = np.array([1, 0, 0]).reshape(1,3)
    R_db_d = rotation_matrix_from_vectors(drillAxis, normal)
    print('RANSAC Plane Fitting R_db_d:\n', R_db_d)
    return R_db_d

def axisCalibration_CylinderFitting(translation_array):
    from py_cylinder_fitting import BestFitCylinder
    from skspatial.objects import Points

    best_fit_cylinder = BestFitCylinder(Points(translation_array))
    p = best_fit_cylinder.point
    axis = best_fit_cylinder.vector
    radius = best_fit_cylinder.radius
    unit_axis = axis/ np.linalg.norm(axis)
    unit_axis = unit_axis.reshape(1,3)

    drillAxis = np.array([1, 0, 0]).reshape(1,3)

    R_db_d = rotation_matrix_from_vectors(drillAxis, unit_axis)
    print('Cylinder Fitting R_db_d:\n', R_db_d)
    return R_db_d

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

if __name__ == '__main__':
    pose_path = sys.argv[1]
    translation_array, F_array = getNdarrayInv(pose_path)
    # translation_array, F_array = getNdarray(pose_path)
    ## oppsite motion
    # translation_array = translation_array[::-1,:]
    # dislist = getNorm(translation_array)
    # visulizePoints(translation_array)
    # visulizeFrames(F_array)
    # axisCalibration_Kabsch(pose_path, translation_array)

    # R_db_d = axisCalibration_RANSAC(translation_array, F_array)
    # r = Rot.from_matrix(R_db_d)
    # rot_vec = r.as_rotvec(degrees=True)
    # print('rotVec:', rot_vec)
    
    # vec = np.array([180, 1, -0.2])
    # r = Rot.from_rotvec(vec)
    # R_db_d = r.as_matrix()
    # print(R_db_d)
    
    R_db_d = axisCalibration_CylinderFitting(translation_array, F_array)
    np.save('../params/R_db_d_.npy', R_db_d)