# Low-level PID control of velocity and attitude
import numpy as np
import matplotlib.pyplot as plt
from pid_control import pid_velocity_fixed_height_controller as pid
import matplotlib.pyplot as plt
import pandas as pd

class kalman_filter():
    def __init__(self):
        self.noise_std_GPS = 0.10
        self.noise_std_ACCEL = 0.01
        self.bias_GPS = 0.0
        self.bias_ACCEL = 0.0

        self.x_noisy_global_last = 0.0
        self.y_noisy_global_last = 0.0
        self.z_noisy_global_last = 0.0

        #Tuning parameter
        self.q_tr = 0.5

        #Initialize KF state and model uncertainty
        self.initialize_KF()

        self.raw_data_vec = []
        self.noisy_data_vec = []
        self.KF_estimate_vec = []

    def add_noise(self, raw_sensor_data, dt, accel_bias = 0.0, gps_bias = 0.0, rng_bias = 0.0):
        noisy_sensor_data = raw_sensor_data.copy()

        #Add Gaussian noise with specified noise parameters to sensor data
        noisy_sensor_data['x_global'] += np.random.normal(loc = gps_bias, scale = self.noise_std_GPS)
        noisy_sensor_data['y_global'] += np.random.normal(loc = gps_bias, scale = self.noise_std_GPS)
        noisy_sensor_data['z_global'] += np.random.normal(loc = gps_bias, scale = self.noise_std_GPS)

        v_x_noisy = (noisy_sensor_data['x_global'] - self.x_noisy_global_last)/dt
        v_y_noisy = (noisy_sensor_data['y_global'] - self.y_noisy_global_last)/dt
        v_z_noisy = (noisy_sensor_data['z_global'] - self.z_noisy_global_last)/dt
        self.x_noisy_global_last = noisy_sensor_data['x_global']
        self.y_noisy_global_last = noisy_sensor_data['y_global']
        self.z_noisy_global_last = noisy_sensor_data['z_global']

        noisy_sensor_data['v_forward'] =  v_x_noisy * np.cos(noisy_sensor_data['yaw']) + v_y_noisy * np.sin(noisy_sensor_data['yaw'])
        noisy_sensor_data['v_left'] = -v_x_noisy * np.sin(noisy_sensor_data['yaw']) + v_y_noisy * np.cos(noisy_sensor_data['yaw'])
        noisy_sensor_data['v_down'] = v_z_noisy
        # noisy_sensor_data['range_down'] = raw_sensor_data['range_down'] + np.random.normal(rng_bias, self.noise_std_RNG)

        noisy_sensor_data['ax_global'] += np.random.normal(loc = accel_bias, scale = self.noise_std_ACCEL)
        noisy_sensor_data['ay_global'] += np.random.normal(loc = accel_bias, scale = self.noise_std_ACCEL)
        noisy_sensor_data['az_global'] += np.random.normal(loc = accel_bias, scale = self.noise_std_ACCEL) 

        return noisy_sensor_data
    
    def initialize_KF(self):
        # Initialize the state vector (X_0) and the covariance matrix (P_0) of the state estimate
        # Assume the state definition in the order: X = [x, v_x, a_x, y, v_y, a_y, z, v_z, a_z], Shape: (9,1), n_states = 9

        # YOUR CODE HERE

        self.X_opt = np.random.rand(9,1)
        self.P_opt = 1000.0*np.diag(np.ones(9))

        # Define the Measurement Matrices (H) for both GPS and ACCELEROMETER measurements - Shape: (n_measurements x n_states)
        self.H_GPS = np.array([[1,0,0,0,0,0,0,0,0],
                               [0,0,0,1,0,0,0,0,0],
                               [0,0,0,0,0,0,1,0,0]
                              ])
        self.H_ACCEL = np.array([[0,0,1,0,0,0,0,0,0],
                                 [0,0,0,0,0,1,0,0,0],
                                 [0,0,0,0,0,0,0,0,1]])

        # Define the Measurement Covariance Matrices (R) for both GPS and ACCELEROMETER measurements - Shape: (n_measurements x n_measurements)
        self.R_GPS = (self.noise_std_GPS**2)*np.eye(3)
        self.R_ACCEL = (self.noise_std_ACCEL**2)*np.eye(3)

    def KF_state_propagation(self, dt):
        # Function that propagates the last fused state over a time-interval dt
        # Inputs:
        #   dt: Time-interval for propagation
        # Outputs:
        #   X_pred: Predicted state after a progpagation time dt (n_states x 1)
        #   P_pred: Predicted covariance after a propagation time dt (n_states x n_states)

        Q_trans = self.calculate_Q(dt, self.q_tr)

        # YOUR CODE HERE -----------------------------------------------------

        # Define the state transition matrix A_trans (n_states x n_states)
        A_trans_sub = np.array([[1, dt, np.power(dt,2)/2],
                                [0, 1, dt],
                                [0, 0, 1]
                               ])
        A_trans = np.block([[A_trans_sub, np.zeros((3,6))],
                            [np.zeros((3,3)), A_trans_sub, np.zeros((3,3,))],
                            [np.zeros((3,6)), A_trans_sub]])
        
        # Calculate the propagated state (X_pred) and the propagated covariance (P_pred) using the last fused state (self.X_opt) and covariance (self.P_opt)
        X_pred = A_trans @ self.X_opt
        P_pred = A_trans @ self.P_opt @ A_trans.transpose() + Q_trans

        return X_pred, P_pred

    def KF_sensor_fusion(self, X_pred, P_pred, H, R, Z):
        # Function that performs sensor fusion when a measurement is received
        # Inputs:
        #   X_pred: State propagated to time of fusion (n_states x 1)
        #   P_pred: Covariance matrix propagated to time of fusion (n_states x n_states)
        #   H: Measurement Matrix of measured sensor (n_measurements x n_states)
        #   R: Measurement Covariance of measured sensor (n_measurements x 9)
        #   Z: Measurement vector received from the sensor (n_measurements x 1)
        # Returns:
        #   self.X_opt: Fused state estimate at sensor readout time (n_states x 1)
        #   self.P_opt: Fused covariance matrix at sensor readout time (n_states x n_states)

        # YOUR CODE HERE
        K = P_pred @ H.transpose() @ (np.linalg.inv(((H @ P_pred @ H.transpose()) + R)))
        self.X_opt = X_pred + K @ (Z - (H @ X_pred))
        self.P_opt = ((np.eye(9)) - K @ H) @ P_pred

        return self.X_opt, self.P_opt

    def KF_estimate(self, measured_state_gps, measured_state_accel, dt_gps, dt_accel, dt_state_prop, sensor_state_flag):
        # Function that outputs the state estimate wehn requested
        # Inputs:
        #   dt_accel: Time elapsed since last acceleration measurements received from accelerometer (always > 0)
        #   dt_gps: Time elapsed since last position measurements received from GPS (always > 0)
        #   dt_state_prop: Time elapsed since last state propagation (Used to predict state when no measurements are recorded) (always > 0)
        #   sensor_state_flag: Possible values are [0,1,2,3]
        #       -> 0: No sensor measurement received at current requested time // 
        #       -> 1: GPS measurement received at current requested time // 
        #       -> 2: Accelerometer measurement received at current requested time
        #       -> 3: Both measurements obtained. Default implementation uses GPS as ground truth
        #   measured_state_gps: The latest GPS position measurement (X,Y,Z) in inertial world frame (n_measurements x 1)
        #   measured_state_accel: The latest ACCELEROMETER measurement (A_X, A_Y, A_Z) in inertial world frame (n_measurements x 1)
        # Returns:
        #   X_est: Estimated drone state (9 x 1)
        #   P_est: Estimated covariance (9 x 9)

        # YOUR CODE HERE

        if sensor_state_flag == 0:
            X_est, P_est = self.KF_state_propagation(dt_state_prop)
            # print("In PROP step")
        if sensor_state_flag == 1 or sensor_state_flag == 3:
            H = self.H_GPS
            R = self.R_GPS
            Z = measured_state_gps
            X_pred_accel, P_pred_accel = self.KF_state_propagation(dt_accel)
            X_est, P_est = self.KF_sensor_fusion(X_pred_accel,P_pred_accel,H,R,Z)
            # print("In GPS meas step")
        if sensor_state_flag == 2:
            H = self.H_ACCEL
            R = self.R_ACCEL
            Z = measured_state_accel
            X_pred_gps, P_pred_gps = self.KF_state_propagation(dt_gps)
            X_est, P_est = self.KF_sensor_fusion(X_pred_gps,P_pred_gps,H,R,Z)
            # print("In ACCEL meas step")

        return X_est, P_est

    def calculate_Q(self, dt_tr, q_tr):
        Q_sub = q_tr * np.array([[np.power(dt_tr,5)/20, np.power(dt_tr,4)/8, np.power(dt_tr,3)/6],
                                   [np.power(dt_tr,4)/8, np.power(dt_tr,3)/3, np.power(dt_tr,2)/2],
                                   [np.power(dt_tr,3)/6, np.power(dt_tr,2)/2, dt_tr]
                                ])
        
        Q = np.block([[Q_sub, np.zeros((3,6))],
                      [np.zeros((3,3)), Q_sub, np.zeros((3,3,))],
                      [np.zeros((3,6)), Q_sub]])
        
        return Q
    
    def aggregate_states(self, raw_data, noisy_data, KF_data):
        keys = ['x_global', 'y_global', 'z_global', 'v_forward', 'v_left', 'ax_global', 'ay_global' , 'az_global']
        self.raw_data_vec.append(list(raw_data[key] for key in keys))
        self.noisy_data_vec.append(list(noisy_data[key] for key in keys))
        self.KF_estimate_vec.append(list(KF_data[key] for key in keys))

    def plot_states(self):
        plt.figure(0)

        raw_data_vec_np = np.array(self.raw_data_vec)
        noisy_data_vec_np = np.array(self.noisy_data_vec)
        KF_estimate_vec_np = np.array(self.KF_estimate_vec)

        plt.plot(raw_data_vec_np[:,:3])
        plt.plot(noisy_data_vec_np[:,:3])
        plt.plot(KF_estimate_vec_np[:,:3])

        plt.figure(1)
        plt.plot(raw_data_vec_np[:,3:5])
        plt.plot(noisy_data_vec_np[:,3:5])
        plt.plot(KF_estimate_vec_np[:,3:5])

        plt.figure(2)
        plt.plot(raw_data_vec_np[:,5:8])
        plt.plot(noisy_data_vec_np[:,5:8])
        plt.plot(KF_estimate_vec_np[:,5:8])

        plt.show()

    # def moving_average(self,data,window_size):
    #     # Define the kernel for the moving average
    #     kernel = np.ones(window_size) / window_size

    #     # Use 'same' mode to ensure the output has the same length as the input
    #     smoothed_data = np.convolve(data, kernel, mode='same')
    #     return smoothed_data