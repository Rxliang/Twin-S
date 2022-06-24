
from dq_calibration.dual_quaternion_hand_eye_calibration import compute_hand_eye_calibration_RANSAC, compute_hand_eye_calibration_BASELINE
from dq_calibration.dual_quaternion import DualQuaternion
import numpy as np
from dq_calibration.algorithm_config import get_exhaustive_search_scalar_part_inliers_config, get_baseline_config, \
    get_exhaustive_search_pose_inliers_config, get_RANSAC_classic_config, get_RANSAC_scalar_part_inliers_config
from dq_calibration.calibration_verification import evaluate_calibration
class HandEyeConfig:

  def __init__(self):

    # General config.
    self.algorithm_name = ""
    self.use_baseline_approach = False
    self.min_num_inliers = 10
    self.enable_exhaustive_search = False

    # Select distinctive poses based on skrew axis
    self.prefilter_poses_enabled = True
    self.prefilter_dot_product_threshold = 0.975

    # RANSAC
    self.ransac_sample_size = 3
    self.ransac_sample_rejection_scalar_part_equality_tolerance = 1e-2
    self.ransac_max_number_iterations = 20
    self.ransac_enable_early_abort = True
    self.ransac_outlier_probability = 0.5
    self.ransac_success_probability_threshold = 0.99
    self.ransac_inlier_classification = "scalar_part_equality"
    self.ransac_position_error_threshold_m = 0.02
    self.ransac_orientation_error_threshold_deg = 1.0

    # Model refinement
    self.ransac_model_refinement = True
    self.ransac_evaluate_refined_model_on_inliers_only = False

    # Hand-calibration
    self.hand_eye_calibration_scalar_part_equality_tolerance = 4e-2

    # Visualization
    self.visualize = False
    self.visualize_plot_every_nth_pose = 10

def pose_inv(pose):
    """
    Inverse of a homogenenous transformation.
    Args:
    - pose (4x4 numpy array)
    Return:
    - inv_pose (4x4 numpy array)
    """
    R = pose[:3, :3]
    t = pose[:3, 3]

    inv_R = R.T
    inv_t = - np.dot(inv_R, t)

    inv_pose = np.c_[inv_R, np.transpose(inv_t)]
    inv_pose = np.r_[inv_pose, [[0, 0, 0, 1]]]

    return inv_pose

def align_w(dq_temp):
    if dq_temp.q_rot.w < 0:
        dq_temp.dq = -dq_temp.dq.copy()
    return dq_temp.copy()

def hand_eye_calibration(pose_W_E, pose_B_H):
    dq_B_H_vec = []
    dq_W_E_vec = []
    # pose_W_E = np.load('A_cam2marker_39_panBase.npy')
    # pose_B_H = np.load('B_base2hand_39_panBase.npy')
    pose_W_E[:, :, :][:3, 3] = pose_W_E[:,:,:][:3,3] * 0.001
    pose_B_H[:, :, :][:3, 3] = pose_B_H[:, :, :][:3, 3] * 0.001

    # pose_W_E = pose_W_E[:,:, :400]
    # pose_B_H = pose_B_H[:,:, :400]

    pose_num = pose_B_H.shape[2]
    for i in range(pose_num):
        dq_B_H = DualQuaternion.from_transformation_matrix(pose_B_H[:, :, i])
        dq_W_E = DualQuaternion.from_transformation_matrix(pose_inv(pose_W_E[:, :, i]))
        # dq_B_H = align_w(dq_B_H)
        # dq_W_E = align_w(dq_W_E)
        dq_B_H_vec.append(dq_B_H)
        dq_W_E_vec.append(dq_W_E)
    # print(dq_B_H_vec[3])
    print('Quaternion loaded!', len(dq_B_H_vec))
    # print("dq_H_E ground truth: \n{}".format(dq_H_E))

    (_, hand_eye_config) = get_exhaustive_search_scalar_part_inliers_config()
    # (_, hand_eye_config) = get_RANSAC_scalar_part_inliers_config(prefilter_poses=True)
    # (_, hand_eye_config) = get_RANSAC_classic_config(prefilter_poses=True)

    (success, dq_H_E_estimated, rmse,
     num_inliers, num_poses_kept,
     runtime, singular_values,
     bad_singular_values) = compute_hand_eye_calibration_RANSAC(
        dq_B_H_vec, dq_W_E_vec, hand_eye_config)
    assert success, "Hand-eye calibration, failed!"

    # pose_H_E_estimated_dq = dq_H_E_estimated
    pose_H_E_estimated = dq_H_E_estimated.to_pose()
    # dq_H_E_estimated.normalize()

    assert pose_H_E_estimated[6] > 0.0, (
        "The real part of the pose's quaternion should be positive. "
        "The pose is: \n{}\n where the dual quaternion was: "
        "\n{}".format(pose_H_E_estimated, dq_H_E_estimated))

    # print("The static input pose was: \n{}".format(pose_H_E))
    print("The hand-eye calibration's output pose is: \n{}".format(
        pose_H_E_estimated))

    # print("T_H_E ground truth: \n{}".format(dq_H_E.to_matrix()))
    T_H_E_estimated = dq_H_E_estimated.to_matrix()
    T_H_E_estimated[:3, 3] = T_H_E_estimated[:3, 3] * 1000
    print("T_H_E estimated: \n{}".format(T_H_E_estimated))
    # np.save('T_H_E_310', T_H_E_estimated)
    return T_H_E_estimated

def hand_eye_calib_2():
    #ransac config
    # hand_eye_config = HandEyeConfig()
    # hand_eye_config.visualize = False
    # hand_eye_config.ransac_max_number_iterations = 50
    # hand_eye_config.ransac_sample_size = 3

    (_, hand_eye_config) = get_baseline_config(False)  # Change here

    # poses = [[1, 2, 3, 1.0, 0., 0., 0.],
    #              [1, -2, -3, -0.5 * math.sqrt(2), 0., 0., -0.5 * math.sqrt(2)]]
    # pose = poses[0]
    # dq = DualQuaternion.from_pose_vector(pose)
    # pose_out = dq.to_pose()
    # print(pose,pose_out)
    poses_W_E = np.load("A_cam2marker_39_panBase.npy")

    poses_B_H = np.load("B_base2hand_39_panBase.npy")

    # for i in range(poses_W_E.shape[2]):
    #    poses_W_E[:,:,i] = pose_inv(poses_W_E[:,:,i])
    # poses_B_H[:,:,i] = pose_inv(poses_B_H[:,:,i])

    print("Shape Check:", poses_W_E.shape)
    print("Converting...")
    # print(poses_B_H[:,:,0])
    size = poses_B_H.shape[2]
    # for i in range(poses_B_H.shape[2]):
    dual_quat_B_H_vec = [DualQuaternion.from_transformation_matrix(poses_B_H[:, :, i]) for i in range(size)]
    print(np.linalg.norm(dual_quat_B_H_vec[0].screw_axis()[0]))

    dual_quat_W_E_vec = [DualQuaternion.from_transformation_matrix(poses_W_E[:, :, i]) for i in range(size)]

    # temp_B_H = [dual_quat_B_H_vec[i].to_pose() for i in range(size)]
    # temp_W_E = [dual_quat_W_E_vec[i].to_pose() for i in range(size)]

    # for i in range(size):
    #    temp_W_E[i][2:] /= np.linalg.norm(temp_W_E[i][2:])
    #    temp_B_H[i][2:] /= np.linalg.norm(temp_B_H[i][2:])
    # # print(len(dual_quat_B_H_vec))
    # print(temp_W_E[0])
    # dual_quat_B_H_vec = [DualQuaternion.from_pose_vector(temp_B_H[i]) for i in range(size)]
    # dual_quat_W_E_vec = [DualQuaternion.from_pose_vector(temp_W_E[i]) for i in range(size)]

    (success, dq_H_E, rmse,
     num_inliers, num_poses_kept,
     runtime, singular_values, bad_singular_values) = compute_hand_eye_calibration_BASELINE(dual_quat_B_H_vec,
                                                                                            dual_quat_W_E_vec,
                                                                                            hand_eye_config)
    # (dq_H_E, s, bad_singular_values)= compute_hand_eye_calibration(
    #         dual_quat_B_H_vec, dual_quat_W_E_vec)

# if __name__ == '__main__':
#     hand_eye_calibration()
#     # hand_eye_calib_2()




