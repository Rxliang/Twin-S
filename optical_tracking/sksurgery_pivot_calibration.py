## Using sksurgery pivot calibraiton
import sys
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
sys.path.append('../')
sys.path.insert(0, '/home/shc/Twin-S/util')
import numpy as np
import random
from scipy.optimize import least_squares
from dataLoader import dataLoader
import pandas as pd
import seaborn as sns
from Solver import solver
sol = solver()
ld = dataLoader()


def _calculate_residual_sphere(parameters, x_values, y_values, z_values):
  """
  Calculates the residual error for an x,y,z coordinates, fitted
  to a sphere with centre and radius defined by the parameters tuple
  :param: A tuple of the parameters to be optimised, should contain
          [x_centre, y_centre, z_centre, radius]
  :param: arrays containing the x,y, and z coordinates.
  :return: The residual error
  """
  # extract the parameters
  x_centre, y_centre, z_centre, radius = parameters

  # use numpy's sqrt function here, which works by element on arrays
  distance_from_centre = np.sqrt((x_values - x_centre) ** 2 +
                                    (y_values - y_centre) ** 2 +
                                    (z_values - z_centre) ** 2)

  return distance_from_centre - radius


def fit_sphere_least_squares(coordinates, initial_parameters,
                             bounds=((-np.inf, -np.inf, -np.inf, -np.inf), (np.inf, np.inf, np.inf, np.inf))):
    """
    Uses scipy's least squares optimisor to fit a sphere to a set
    of 3D Points
    :param coordinates: (x,y,z) n x 3 array of point coordinates
    :param initial parameters: 1 x 4  array containing four initial
        values (centre, and radius)
    :return: x: an array containing the four fitted parameters
    :return: ier: int An integer flag. If it is equal to 1, 2, 3 or 4, the
             solution was found.
    """
    x_values = coordinates[:, 0]
    y_values = coordinates[:, 1]
    z_values = coordinates[:, 2]
    return least_squares(_calculate_residual_sphere, initial_parameters,
                         bounds=bounds,
                         method='trf',
                         jac='3-point',
                         args=(x_values, y_values, z_values))

def pivot_calibration(tracking_matrices, configuration=None):
    """
    Performs pivot calibration on an array of tracking matrices
    :param tracking_matrices: an Nx4x4 array of tracking matrices
    :param configuration: an optional configuration dictionary, if not
        the algorithm defaults to Algebraic One Step. Other options
        include ransac, and sphere_fitting
    :returns: tuple containing;
            'pointer_offset' The coordinate of the pointer tip relative to
            the tracking centre
            'pivot_point' The location of the pivot point in world coordinates
            'residual_error' The RMS pointer tip error, errors in
            each direction are treated as independent variables, so for a
            calibration with n matrices, RMS error is calculated using
            nx3 measurements.
    :raises: TypeError, ValueError
    """

    if not isinstance(tracking_matrices, np.ndarray):
        raise TypeError("tracking_matrices is not a numpy array'")

    if not tracking_matrices.shape[1] == 4:
        raise ValueError("tracking_matrices should have 4 rows per matrix")

    if not tracking_matrices.shape[2] == 4:
        raise ValueError("tracking_matrices should have 4 columns per matrix")

    use_algebraic_one_step = False
    use_ransac = False
    use_sphere_fitting = False

    if configuration is not None:
        use_algebraic_one_step = False
        method = configuration.get('method', 'aos')
        if method == 'aos':
            use_algebraic_one_step = True
        elif method == 'ransac':
            use_ransac = True
        elif method == 'sphere_fitting':
            use_sphere_fitting = True
    else:
        use_algebraic_one_step = True

    if use_algebraic_one_step:
        return pivot_calibration_aos(tracking_matrices)

    if use_ransac:
        number_iterations = configuration.get('number_iterations', 10)
        error_threshold = configuration.get('error_threshold', 4)
        consensus_threshold = configuration.get('consensus_threshold',
                                                0.25)
        early_exit = configuration.get('early_exit', False)
        return pivot_calibration_with_ransac(
            tracking_matrices, number_iterations, error_threshold,
            consensus_threshold, early_exit)

    if use_sphere_fitting:
        init_parameters = configuration.get('init_parameters', None)
        return pivot_calibration_sphere_fit(tracking_matrices, init_parameters)

    raise ValueError("method key set to unknown method; ",
                     configuration.get('method', 'aos'))


def pivot_calibration_aos(tracking_matrices):

    """
    Performs Pivot Calibration, using Algebraic One Step method,
    and returns Residual Error.
    See `Yaniv 2015 <https://dx.doi.org/10.1117/12.2081348>`_.
    :param tracking_matrices: N x 4 x 4 ndarray, of tracking matrices.
    :returns: pointer offset, pivot point and RMS Error about centroid of pivot.
    :raises: ValueError if rank less than 6
    """

    # See equation in section 2.1.2 of Yaniv 2015.
    # Ax = b.

    a_values, b_values = _matrices_to_a_and_b(tracking_matrices)

    # To calculate Singular Value Decomposition

    u_values, s_values, v_values = np.linalg.svd(a_values, full_matrices=False)
    c_values = np.dot(u_values.T, b_values)
    w_values = np.dot(np.diag(1 / s_values), c_values)
    x_values = np.dot(v_values.T, w_values)

    # Calculating the rank, and removing close to zero singular values.
    rank = _replace_small_values(s_values, 0.01, 0.0)

    if rank < 6:
        raise ValueError("PivotCalibration: Failed. Rank < 6")

    pointer_offset = x_values[0:3]
    pivot_location = x_values[3:6]
    residual_error = _residual_error_direct(a_values, b_values, x_values)

    return pointer_offset, pivot_location, residual_error


def pivot_calibration_with_ransac(tracking_matrices,
                                  number_iterations,
                                  error_threshold,
                                  concensus_threshold,
                                  early_exit=False
                                  ):
    """
    Written as an exercise for implementing RANSAC.
    :param tracking_matrices: N x 4 x 4 ndarray, of tracking matrices.
    :param number_iterations: the number of iterations to attempt.
    :param error_threshold: distance in millimetres from pointer position
    :param concensus_threshold: the minimum percentage of inliers to finish
    :param early_exit: If True, returns model as soon as thresholds are met
    :returns: pointer offset, pivot point and RMS Error about centroid of pivot.
    :raises: TypeError, ValueError
    """
    if number_iterations < 1:
        raise ValueError("The number of iterations must be > 1")
    if error_threshold < 0:
        raise ValueError("The error threshold must be a positive distance.")
    if concensus_threshold < 0 or concensus_threshold > 1:
        raise ValueError("The concensus threshold must be [0-1] as percentage")
    if not isinstance(tracking_matrices, np.ndarray):
        raise TypeError("tracking_matrices is not a numpy array'")

    number_of_matrices = tracking_matrices.shape[0]
    population_of_indices = range(number_of_matrices)
    minimum_matrices_required = 3

    highest_number_of_inliers = -1
    best_pointer_offset = None
    best_pivot_location = None
    best_residual_error = -1

    for iter_counter in range(number_iterations):
        indexes = random.sample(population_of_indices,
                                minimum_matrices_required)
        sample = tracking_matrices[indexes]

        try:
            pointer_offset, pivot_location, _ = pivot_calibration(sample)
        except ValueError:
            print("RANSAC, iteration " + str(iter_counter) + ", failed.")
            continue

        # Need to evaluate the number of inliers.
        # Slow, but it's written as a teaching exercise.
        number_of_inliers = 0
        inlier_indices = []
        for matrix_counter in range(number_of_matrices):
            offset = np.vstack((pointer_offset, 1))
            transformed_point = tracking_matrices[matrix_counter] @ offset
            diff = pivot_location - transformed_point[0:3]
            norm = np.linalg.norm(diff)
            if norm < error_threshold:
                number_of_inliers = number_of_inliers + 1
                inlier_indices.append(matrix_counter)

        percentage_inliers = number_of_inliers / number_of_matrices

        # Keep the best model so far, based on the highest number of inliers.
        if percentage_inliers > concensus_threshold \
                and number_of_inliers > highest_number_of_inliers:
            highest_number_of_inliers = number_of_inliers
            inlier_matrices = tracking_matrices[inlier_indices]
            best_pointer_offset, best_pivot_location, best_residual_error = \
                pivot_calibration(inlier_matrices)

        # Early exit condition, as soon as we find model with enough fit.
        if percentage_inliers > concensus_threshold and early_exit:
            return best_pointer_offset, best_pivot_location, best_residual_error

    if best_pointer_offset is None:
        raise ValueError("Failed to find a model using RANSAC.")

    print("RANSAC Pivot, from " + str(number_of_matrices)
          + " matrices, used " + str(highest_number_of_inliers)
          + " matrices, with error threshold = " + str(error_threshold)
          + " and consensus threshold = " + str(concensus_threshold)
          )

    return best_pointer_offset, best_pivot_location, best_residual_error


def pivot_calibration_sphere_fit(tracking_matrices, init_parameters=None):

    """
    Performs Pivot Calibration, using sphere fitting, based on
    See `Yaniv 2015 <https://dx.doi.org/10.1117/12.2081348>`_.
    :param tracking_matrices: N x 4 x 4 ndarray, of tracking matrices.
    :param init_parameters: 1X4 array of initial parameter for finding the
        pivot point in world coords and pivot radius. Default is to set to
        the mean x,y,z values and radius = 0.
    :returns: pointer offset, pivot point and RMS Error about centroid of pivot.
    """
    residual_error = -1.0
    translations = tracking_matrices[:, 0:3, 3]

    # first find the pivot point in world coordinates using sphere fitting
    if init_parameters is None:
        means = np.mean(translations, 0)
        init_parameters = np.concatenate([means, np.zeros((1))])

    result = fit_sphere_least_squares(translations, init_parameters)
    pivot_point = result.x[0:3]

    # now calculate the mean offset
    rotations = tracking_matrices[:, 0:3, 0:3]
    offsets = np.zeros((tracking_matrices.shape[0], 3))

    for index, rot in enumerate(rotations):
        offsets[index] = rot.transpose() @ (pivot_point - translations[index])

    pointer_offset = np.mean(offsets, 0)

    residual_error = _residual_error(tracking_matrices, pointer_offset,
                                     pivot_point)

    return pointer_offset, pivot_point, residual_error


def _matrices_to_a_and_b(tracking_matrices):
    """
    Helper function to convert tracking matrices into
    a_values and b_values that can be used for aos calibration or
    for calculating residuals.
    :param tracking_matrices: nx4x4 tracking matrices
    :returns: a_values, (nx3)x6 array of rotation and -Identity, b_values,
    an nx3 column vector of translations
    """
    number_of_matrices = tracking_matrices.shape[0]
    # A contains rotation matrix from each tracking matrix.
    # and -I for each tracking matrix.
    size_a = 3 * number_of_matrices, 3
    a_first = (tracking_matrices [:, 0:3, 0:3]).reshape(size_a)
    a_second = (np.eye(3) * -1.0).reshape((1, 3, 3)).repeat(
        number_of_matrices, 0).reshape(size_a)
    a_values = np.concatenate((a_first, a_second), axis=1)

    # Column vector containing -1 * translation from each tracking matrix.
    size_b = 3 * number_of_matrices, 1
    b_values = (tracking_matrices[:, 0:3, 3] * -1.0).reshape((size_b))

    return a_values, b_values


def _residual_error(tracking_matrices, pointer_offset, pivot_location):
    """
    Helper function to calculate resdiual (RMS) errors.
    :params tracking_matrices: nx4x4 array
    :params pointer_offset: 1x3 array
    :params pivot_location: 1x3 array
    :returns: The RMS residual error
    """
    x_values = np.concatenate([pointer_offset, pivot_location],
                              axis=0).reshape((6, 1))
    a_values, b_values = _matrices_to_a_and_b(tracking_matrices)
    return _residual_error_direct(a_values, b_values, x_values)


def _residual_error_direct(a_values, b_values, x_values):
    """
    Helper function to calculate resdidual (RMS) errors.
    :params a_values: (nx3)x6 array of rotation and -Identity,
    :params b_values: an nx3 column vector of translations
    :params x_values: nx6 array, pointer_offset and pivot_point
    :returns: TRhe RMS residual error
    """
    residual_matrix = (np.dot(a_values, x_values) - b_values)
    residual_error = np.mean(residual_matrix * residual_matrix)
    residual_error = np.sqrt(residual_error)
    return residual_error


def _replace_small_values(the_list, threshold=0.01, replacement_value=0.0):
    """
    replace small values in a list, this changes the list in place.
    :param the_list: the list to process.
    :param threshold: replace values lower than threshold.
    :param replacement_value: value to replace with.
    :returns: the number of items not replaced.
    """
    rank = 0
    for index, item in enumerate(the_list):
        if item < threshold:
            the_list[index] = replacement_value
        else:
            rank += 1

    return rank


def rotationEval(tracking_matrices):
    errors = []
    eulers = []
    t_tip_eval = np.load('../params/t_tip_eval.npy')
    p_eval = np.load('../params/p_eval.npy')
    for T in tracking_matrices:
        Rm = T[:3, :3]
        t = T[:3, 3].reshape(3,1)
        p_cal = Rm@t_tip_eval+t
        error = np.linalg.norm(p_eval-p_cal)
        if error < 1:
            r = R.from_matrix(Rm)
            euler = r.as_euler('xyz', degrees=True)
            eulers.append(euler)
            errors.append(error)
    
    eulers = np.array(eulers)
    # plt.subplot(1,3,1)
    # plt.xlabel('degrees')
    # plt.ylabel('error in mm')
    # # plt.hist(errors, bins=20, density=True)
    # plt.scatter(eulers[:,0], errors)
    # plt.subplot(1,3,2)
    # plt.xlabel('degrees')
    # plt.ylabel('error in mm')
    # # plt.hist(errors, bins=20, density=True)
    # plt.scatter(eulers[:,1], errors)
    # plt.subplot(1,3,3)
    # plt.xlabel('degrees')
    # plt.ylabel('error in mm')
    # # plt.hist(errors, bins=20, density=True)
    # plt.scatter(eulers[:,2], errors)

    # plt.subplot(1,2,1)
    # plt.xlabel('degrees')
    # plt.ylabel('error in mm')
    # plt.scatter((np.linalg.norm(eulers, axis=1)-np.mean(np.linalg.norm(eulers, axis=1))), errors)
    X = (np.linalg.norm(eulers, axis=1)-np.mean(np.linalg.norm(eulers, axis=1)))
    sns.set(style="white", color_codes=True)
    sns.jointplot(x=X, y=errors, kind='scatter', s=20, color='b')
    sns.jointplot(x=X, y=errors, kind='kde', color="b")
    # plt.title('rotational error')
    # plt.subplot(1,2,2)
    # plt.xlabel('error in mm')
    # plt.ylabel('density')
    # plt.hist(errors, bins=20, density=True)
    # plt.title('distribution of tracking error impacted by rotation')
    # plt.show()

    from scipy.stats import chi2_contingency
    from scipy.stats import chi2
    from sklearn import preprocessing

    X = X+25
    # table = [[10,20,30],[6,9,17]]
    rot_data =np.vstack([X, errors])
    print(rot_data.shape)
    table = rot_data
    # print(table)
    stat,p,dof,expected = chi2_contingency(table) # stat卡方统计值，p：P_value，dof 自由度，expected理论频率分布
    print('dof=%d'%dof)
    # print(expected)

    prob = 0.95 # 选取95%置信度
    critical = chi2.ppf(prob,dof)  # 计算临界阀值
    print('probality=%.3f,critical=%.3f,stat=%.3f '%(prob,critical,stat))
    if abs(stat)>=critical:
        print('reject H0:Dependent')
    else:
        print('fail to reject H0:Independent')

    alpha = 1-prob
    print('significance=%.3f,p=%.3f'%(alpha,p))
    if p<alpha:
        print('reject H0:Dependent')
    else:
        print('fail to reject H0:Independent')

if __name__ == '__main__':
    ld = dataLoader()
    pose_path = sys.argv[1]
    csv_data = pd.read_csv(pose_path)
    # print(csv_data.head())
    num_frames = len(csv_data)
    tracking_matrices = np.zeros([num_frames, 4, 4])
    for i in range(num_frames):
        # quaternion = ld.getToolPose(i, csv_data)
        quaternion = ld.getToolPose(i, pose_path)
        _, T = sol.seven2trans(quaternion)
        tracking_matrices[i, :, :] = T

    # rotationEval(tracking_matrices)

    t_tip, p,  residual = pivot_calibration_with_ransac(tracking_matrices,
                                  50,
                                  0.1,
                                  0.1,
                                  early_exit=False
                                  )
    print('t_tip:', t_tip.T, "\np:", p.T, '\nresidual:', residual)
    save_name = sys.argv[2]
    np.save('../params/' + str(save_name) + '.npy', t_tip)
    # np.save('../params/' + 't_tip_eval.npy', t_tip)
    # np.save('../params/' + 'p_eval.npy', p)

    # t_tip: [[  12.83037253 -168.75504173   56.3511996 ]] 
    # t_tip: [[  12.86018514 -171.22484561   56.06308705]] 1023
    # t_tip: [[  14.60067685 -175.91251655   56.08049718]] 0205
    # t_tip: [[  14.81979456 -174.32136278   57.22970104]] 0220

