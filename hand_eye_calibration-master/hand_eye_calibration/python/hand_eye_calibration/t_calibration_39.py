
from dual_quaternion_hand_eye_calibration import compute_hand_eye_calibration_RANSAC, compute_hand_eye_calibration_BASELINE
from dual_quaternion import DualQuaternion
import numpy as np

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

def hand_eye_calibration():
    pose_W_E = np.load('A_cam2marker_37_panBase.npy')
    pose_B_H = np.load('B_base2hand_37_panBase.npy')
    pose_num = len(pose_B_H)
    # dq_B_H_vec, dq_W_E_vec = he_helpers.generate_test_paths(
    #     20, self.dq_H_E, self.dq_B_W, self.paths_start_at_origin)
    dq_B_H_vec = [DualQuaternion.from_transformation_matrix(pose_B_H[:, :, i]) for i in range(pose_num)]
    dq_W_E_vec = [DualQuaternion.from_transformation_matrix(pose_W_E[:, :, i]) for i in range(pose_num)]
    print('Quaternion loaded!')
    # print("dq_H_E ground truth: \n{}".format(dq_H_E))

    hand_eye_config = HandEyeConfig()
    hand_eye_config.visualize = False
    hand_eye_config.ransac_max_number_iterations = 50
    hand_eye_config.ransac_sample_size = 3
    (success, dq_H_E_estimated, rmse,
     num_inliers, num_poses_kept,
     runtime, singular_values,
     bad_singular_values) = compute_hand_eye_calibration_BASELINE(
        dq_B_H_vec, dq_W_E_vec, hand_eye_config)
    assert success, "Hand-eye calibration, failed!"

    pose_H_E_estimated = dq_H_E_estimated.to_pose()
    dq_H_E_estimated.normalize()

    assert pose_H_E_estimated[6] > 0.0, (
        "The real part of the pose's quaternion should be positive. "
        "The pose is: \n{}\n where the dual quaternion was: "
        "\n{}".format(pose_H_E_estimated, dq_H_E_estimated))

    # print("The static input pose was: \n{}".format(pose_H_E))
    print("The hand-eye calibration's output pose is: \n{}".format(
        pose_H_E_estimated))

    # print("T_H_E ground truth: \n{}".format(dq_H_E.to_matrix()))
    print("T_H_E estimated: \n{}".format(dq_H_E_estimated.to_matrix()))

    # assert np.allclose(
    #     dq_H_E.dq, dq_H_E_estimated.dq, atol=1e-3), (
    #     "input dual quaternion: {}, estimated dual quaternion: {}".format(
    #         dq_H_E, dq_H_E_estimated))

if __name__ == '__main__':
    hand_eye_calibration()