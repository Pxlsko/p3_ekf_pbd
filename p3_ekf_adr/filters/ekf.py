
import numpy as np

import time

class ExtendedKalmanFilter:

	def __init__(self, initial_state, initial_covariance, motion_model, observation_model, **kwargs):
		# Process arguments
		proc_noise_std = kwargs.get('proc_noise_std', [0.02, 0.02, 0.01])
		obs_noise_std = kwargs.get('obs_noise_std', [0.02, 0.02, 0.01])

		self.mu = initial_state # Initial state estimate 
		self.Sigma = initial_covariance # Initial uncertainty

		self.g, self.G, self.V = motion_model() # The action model to use.
		
		# Standard deviations of the process or action model noise
		self.proc_noise_std = np.array(proc_noise_std)
		# Process noise covariance (R)
		self.R = np.diag(self.proc_noise_std ** 2)

		self.h, self.H = observation_model() # The observation model to use

		# Standard deviations for the observation or sensor model noise
		self.obs_noise_std = np.array(obs_noise_std)
		# Observation noise covariance (Q)
		self.Q = np.diag(self.obs_noise_std ** 2)

		self.exec_times_pred = []
		self.exec_times_upd = []

		
	def predict(self, u, dt):
			start_time = time.time()

			# 1. Calcular nueva estimación de estado usando función de transición g
			self.mu = self.g(self.mu, u, dt)

			# 2. Calcular Jacobiano de la función g respecto al estado
			G_t = self.G(self.mu, u, dt)

			# 3. Calcular Jacobiano de la función g respecto a la entrada de control (no usado aquí directamente)
			V = self.V(self.mu, u, dt)

			# 4. Actualizar la matriz de covarianza con la propagación del error y ruido de proceso
			self.Sigma = G_t @ self.Sigma @ G_t.T + self.R

			end_time = time.time()
			execution_time = end_time - start_time
			self.exec_times_pred.append(execution_time)

			return self.mu, self.Sigma

	def update(self, z, dt):
		start_time = time.time()

		# 1. Calcular Jacobiano de la función de observación H_t en el estado actual
		H_t = self.H(self.mu)

		# 2. Calcular la covarianza de la innovación
		S = H_t @ self.Sigma @ H_t.T + self.Q

		# 3. Calcular la ganancia de Kalman K
		K = self.Sigma @ H_t.T @ np.linalg.inv(S)

		# 4. Calcular la innovación (diferencia entre observación real y estimada)
		z_vec = z.reshape(-1)
		h_vec = self.h(self.mu).reshape(-1)
		y = z_vec - h_vec

		# 5. Actualizar la estimación del estado usando la ganancia de Kalman y la innovación
		self.mu = self.mu + K @ y

		# 6. Actualizar la matriz de covarianza de la estimación
		I = np.eye(len(self.mu))
		self.Sigma = (I - K @ H_t) @ self.Sigma

		end_time = time.time()
		self.exec_times_upd.append(end_time - start_time)
		return self.mu, self.Sigma
