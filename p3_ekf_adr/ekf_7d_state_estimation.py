
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

    # Ajuste Balanceado
    #proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1] 
    #obs_noise_std  = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438] 
    # Ajuste con Alta Q
    #proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]  
    #obs_noise_std  = [1000.0, 1000.0, 10000.0, 6.853891945200942e-05, 1.0966227112321507e-05, 0.15, 0.15]  
    # Ajuste con Alta R
    proc_noise_std = [1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0]  
    obs_noise_std  = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]  


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
