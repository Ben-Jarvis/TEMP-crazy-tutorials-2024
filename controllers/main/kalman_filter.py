# Low-level PID control of velocity and attitude
import numpy as np
import matplotlib.pyplot as plt
from pid_control import pid_velocity_fixed_height_controller as pid
import yaml
import pandas as pd

class kalman_filter():
    def __init__(self):
        self.noise_std_GPS = 0.1
        self.noise_std_ACCEL = 0.1
        # self.noise_std_RNG = 0.0

        self.bias_GPS = 0.0
        self.bias_ACCEL = 0.0

        #Tuning parameters
        self.q_tr = 0.0

        #Measurement matrix definition
        self.H_GPS = np.array([[1,0,0,0,0,0,0,0,0],
                               [0,0,0,1,0,0,0,0,0],
                              ])
        self.H_ACCEL = np.array([[0,0,1,0,0,0,0,0,0],
                                 [0,0,0,0,0,1,0,0,0],
                                 [0,0,0,0,0,0,0,0,1]])
        # self.H_RNG = np.array([[0,0,0,0,0,0,1,0,0]])

        #Measurement covariance matrix definition
        self.R_GPS = (self.noise_std_GPS**2)*np.eye(2)
        self.R_ACCEL = (self.noise_std_ACCEL**2)*np.eye(3)

        #Initialize KF state and model uncertainty
        self.initialize_KF()

    def add_noise(self, raw_sensor_data, accel_bias = 0.0, gps_bias = 0.0, rng_bias = 0.0):
        noisy_sensor_data = raw_sensor_data

        #Add Gaussian noise with specified noise parameters to sensor data
        noisy_sensor_data['x_global'] += np.random.normal(loc = gps_bias, scale = self.noise_std_ACCEL)
        noisy_sensor_data['y_global'] += np.random.normal(loc = gps_bias, scale = self.noise_std_GPS)
        # noisy_sensor_data['range_down'] = raw_sensor_data['range_down'] + np.random.normal(rng_bias, self.noise_std_RNG)

        noisy_sensor_data['ax_global'] += np.random.normal(loc = accel_bias, scale = self.noise_std_ACCEL)
        noisy_sensor_data['ay_global'] += np.random.normal(loc = accel_bias, scale = self.noise_std_ACCEL)
        noisy_sensor_data['az_global'] += np.random.normal(loc = accel_bias, scale = self.noise_std_ACCEL) 

        return noisy_sensor_data

    def KF_state_propagation(self, dt):
        Q_trans = self.calculate_Q(dt, self.q_tr)
        A_trans_sub = np.array([[1, dt, np.power(dt,2)/2],
                                [0, 1, dt],
                                [0, 0, 1]
                               ])
        A_trans = np.block([[A_trans_sub, np.zeros((3,6))],
                            [np.zeros((3,3)), A_trans_sub, np.zeros((3,3,))],
                            [np.zeros((3,6)), A_trans_sub]])
        
        X_pred = A_trans @ self.X_opt
        P_pred = A_trans @ self.P_opt @ A_trans.transpose() + Q_trans

        return X_pred, P_pred

    def KF_sensor_fusion_estimate(self, measured_state_gps, measured_state_accel, dt_gps, dt_accel, gps_period, accel_period):
        measurement_obtained = False
        if dt_gps >= gps_period:
            H = self.H_GPS
            R = self.R_GPS
            Z = measured_state_gps
            dt = dt_accel
            measurement_obtained = True
            # print("In 1")
        elif dt_accel >= accel_period:
            H = self.H_ACCEL
            R = self.R_ACCEL
            Z = measured_state_accel
            dt = dt_gps
            measurement_obtained = True
        else:
            dt = min(dt_accel, dt_gps)
            # print("In 2")
            
        X_pred, P_pred = self.KF_state_propagation(dt)

        if measurement_obtained == True:
            K = P_pred @ H.transpose() @ (np.linalg.inv(((H @ P_pred @ H.transpose()) + R)))
            self.X_opt = X_pred + K @ (Z - (H @ X_pred))
            self.P_opt = ((np.eye(9)) - K @ H) @ P_pred
            X_est, P_est = self.X_opt, self.P_opt
        else:
            X_est, P_est = X_pred, P_pred

        return X_est, P_est
    
    def initialize_KF(self):
        # Initialize the state vector (X_0) and the covariance matrix (P_0) of the state estimate
        # Assume the state definition as:
        # X_trans = [x, v_x, a_x, y, v_y, a_y, z, v_z, a_z]
        # X_ang = [roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate]
        self.X_opt = np.random.rand(9,1)
        self.P_opt = 1000.0*np.diag(np.ones(9))

        # self.ang_opt = np.random.rand(6,1)
        # self.P_ang = 1000.0*np.diag(np.ones(6))

    def calculate_Q(self, dt_tr, q_tr):
        Q_sub = q_tr * np.array([[np.power(dt_tr,5)/20, np.power(dt_tr,4)/8, np.power(dt_tr,3)/6],
                                   [np.power(dt_tr,4)/8, np.power(dt_tr,3)/3, np.power(dt_tr,2)/2],
                                   [np.power(dt_tr,3)/6, np.power(dt_tr,2)/2, dt_tr]
                                ])
        
        Q = np.block([[Q_sub, np.zeros((3,6))],
                      [np.zeros((3,3)), Q_sub, np.zeros((3,3,))],
                      [np.zeros((3,6)), Q_sub]])
        
        # Q_ang = q_ang * np.array([[np.power(dt_ang,3)/3, np.power(dt_ang,2)/2],
        #                           [np.power(dt_ang,2)/2, dt_ang]
        #                          ])
        
        return Q
    

    # def moving_average(self,data,window_size):
    #     # Define the kernel for the moving average
    #     kernel = np.ones(window_size) / window_size

    #     # Use 'same' mode to ensure the output has the same length as the input
    #     smoothed_data = np.convolve(data, kernel, mode='same')
    #     return smoothed_data