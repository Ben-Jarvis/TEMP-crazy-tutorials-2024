# Low-level PID control of velocity and attitude
import numpy as np
import matplotlib.pyplot as plt
from pid_control import pid_velocity_fixed_height_controller as pid
import yaml
import pandas as pd

class kalman_filter():
    def __init__(self):
        self.noise_std_GPS = 0.1
        self.noise_std_ACCEL = 0.2

        self.bias_GPS = 0.0
        self.bias_ACCEL = 0.0

        #Tuning parameters
        self.q_tr = 0.1

        #Measurement matrix definition
        self.H_GPS = np.array([[1,0,0,0,0,0,0,0,0],
                               [0,0,0,1,0,0,0,0,0],
                               [0,0,0,0,0,0,1,0,0]])
        self.H_ACCEL = np.array([[0,0,1,0,0,0,0,0,0],
                                 [0,0,0,0,0,1,0,0,0],
                                 [0,0,0,0,0,0,0,0,1]])

        #Measurement covariance matrix definition
        self.R_GPS = self.noise_std_GPS*np.eye(3)
        self.R_ACCEL = self.noise_std_ACCEL*np.eye(3)

        #Initialize KF state and model uncertainty
        self.initialize_KF()


    def add_noise(self, raw_sensor_data, accel_noise_std, gps_noise_std, accel_bias = 0.0, gps_bias = 0.0):
        noisy_sensor_data = {}

        #Add Gaussian noise with specified noise parameters to sensor data
        noisy_sensor_data['x_global'] = raw_sensor_data['x_global'] + np.random.normal(gps_bias, gps_noise_std)
        noisy_sensor_data['y_global'] = raw_sensor_data['y_global'] + np.random.normal(gps_bias, gps_noise_std)

        noisy_sensor_data['roll'] = raw_sensor_data['roll'] + np.random.normal(accel_bias, accel_noise_std)
        noisy_sensor_data['pitch'] = raw_sensor_data['pitch'] + np.random.normal(accel_bias, accel_noise_std)
        noisy_sensor_data['yaw'] = raw_sensor_data['yaw'] + np.random.normal(accel_bias, accel_noise_std) 

        return noisy_sensor_data

    def KF_state_estimate(self, dt):
        Q_trans = self.calculate_Q(dt, self.q_tr)
        A_trans_sub = np.array([[1, dt, np.power(dt,2)/2],
                                [0, 1, dt],
                                [0, 0, 1]
                               ])
        A_trans = np.block([[A_trans_sub, np.zeros((3,6))],
                            [np.zeros((3,3)), A_trans_sub, np.zeros((3,3,))],
                            [np.zeros((3,6)), A_trans_sub]])
        
        X_pred = A_trans @ self.X_opt
        P_pred = A_trans @ self.P @ A_trans.transpose() + Q_trans

        return X_pred, P_pred

    def KF_sensor_fusion(self, measured_state, dt_gps, dt_accel):
        Z = measured_state
        if dt_gps == 0.0:
            H = self.H_ACCEL
            R = self.R_ACCEL
            dt = dt_accel
        elif dt_accel == 0.0:
            H = self.H_GPS
            R = self.R_GPS
            dt = dt_gps
            
        #dt = np.min(dt_gps, dt_accel)
        X_pred, P_pred = self.KF_state_estimate(dt)

        K = P_pred @ H.transpose() / ((H @ P_pred @ H.transpose()) + R)
        self.X_opt = X_pred + K @ (Z - H @ X_pred)
        self.P = ((np.eye(9)) - K @ H) @ P_pred


    def initialize_KF(self):
        # Initialize the state vector (X_0) and the covariance matrix (P_0) of the state estimate
        # Assume the state definition as:
        # X_trans = [x, v_x, a_x, y, v_y, a_y, z, v_z, a_z]
        # X_ang = [roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate]
        self.X_opt = np.random.rand(9,1)
        self.P = 1000.0*np.diag(np.ones(9))

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