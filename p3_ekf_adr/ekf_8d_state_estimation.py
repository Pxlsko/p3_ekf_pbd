import rclpy
from .motion_models.acceleration_motion_models import acceleration_motion_model_linearized_2
from .observation_models.odometry_imu_observation_models import odometry_imu_observation_model_with_acceleration_motion_model_linearized_2
import numpy as np

# TODO: Import the correct motion and observation models for the 8D case

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterFusionNode as ExtendedKalmanFilterFusionNode


def main(args=None):
    # Initialize the Kalman Filter

    mu0 = np.zeros(8)
    Sigma0 = np.eye(8)

    # Ajuste Balanceado
    #proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]  # Ruido moderado en el modelo
    #obs_noise_std  = [100.0, 100.0, 1000.0, 6.85e-06, 1.10e-06, 0.00153, 0.00153]  # Ruido moderado en las observaciones
    # Ajuste con Alta Q
    #proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]  # Ruido moderado en el modelo
    #obs_noise_std  = [1000.0, 1000.0, 10000.0, 6.85e-05, 1.10e-05, 0.15, 0.15]  # Ruido alto en las observaciones
    # Ajuste con Alta R
    proc_noise_std = [1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0]  # Ruido alto en el modelo
    obs_noise_std  = [100.0, 100.0, 1000.0, 6.85e-06, 1.10e-06, 0.00153, 0.00153]  # Ruido moderado en las observaciones


    # === TODO: Replace the None below with proper motion and observation model functions ===
    ekf = ExtendedKalmanFilter(mu0, Sigma0,
                               acceleration_motion_model_linearized_2,
                               odometry_imu_observation_model_with_acceleration_motion_model_linearized_2,
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
