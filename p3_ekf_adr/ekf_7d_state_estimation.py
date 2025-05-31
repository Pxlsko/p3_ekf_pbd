
import rclpy
from .motion_models.acceleration_motion_models import acceleration_motion_model_linearized_1
from .observation_models.odometry_imu_observation_models import odometry_imu_observation_model_with_acceleration_motion_model_linearized_1
import numpy as np

# TODO: Import the correct motion and observation models for the 7D case

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterFusionNode as ExtendedKalmanFilterFusionNode


def main(args=None):
    # Initialize the Kalman Filter

    mu0 = np.zeros(7)
    Sigma0 = np.eye(7)

    # Ajuste balanceado
    #proc_noise_std = [1.0, 1.0, 0.5, 0.2, 0.2, 0.5, 0.5]
    #obs_noise_std = [1.0, 1.0, 0.5, 0.2, 0.2, 0.5, 0.5]
    # Alta Q
    #proc_noise_std = [1.0, 1.0, 0.5, 0.2, 0.2, 0.5, 0.5]
    #obs_noise_std = [2.0, 2.0, 1.0, 0.4, 0.4, 1.0, 1.0]
    # Alta R
    proc_noise_std = [2.0, 2.0, 1.0, 0.4, 0.4, 1.0, 1.0]
    obs_noise_std = [1.0, 1.0, 0.5, 0.2, 0.2, 0.5, 0.5]

    # === TODO: Replace the None below with proper motion and observation model functions ===
    ekf = ExtendedKalmanFilter(mu0, Sigma0,
                               acceleration_motion_model_linearized_1,
                               odometry_imu_observation_model_with_acceleration_motion_model_linearized_1,
                               proc_noise_std=proc_noise_std,
                               obs_noise_std=obs_noise_std)
    # ===================================================================

    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterFusionNode(ekf)
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
