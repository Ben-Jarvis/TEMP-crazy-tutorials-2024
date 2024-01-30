# Low-level PID control of velocity and attitude
import numpy as np
import matplotlib.pyplot as plt

class pid_velocity_fixed_height_controller():
    def __init__(self):
        self.pastVxError = 0
        self.pastVyError = 0
        self.pastAltError = 0
        self.pastPitchError = 0
        self.pastRollError = 0
        self.pastYawrateError = 0

        self.intVx = 0
        self.intVy = 0
        self.intAlt = 0
        self.intPitch = 0
        self.intRoll = 0
        self.intYawrate = 0

        self.global_time = 0

        # Only for tuning
        self.tuning_level = "off"
        self.tuning_on = False
        self.tuning_start = 7
        self.tuning_iter = 2
        self.tuning_time = 0.0
        self.tuning_ts = []
        self.tuning_desired = []
        self.tuning_actual = []

    def pid(self, dt, action, actual_roll, actual_pitch, actual_yaw_rate,
            actual_alt, actual_vx, actual_vy):

        # Bad gains
        gains = {"off_alt": 55.0,   "kp_alt": 4.0,        "ki_alt": 0.5,        "kd_alt": 5.0,
                                    "kp_att_rp": 0.5,     "ki_att_rp":0.0,      "kd_att_rp": 0.1,
                                    "kp_att_y": 1.0,      "ki_att_y": 0.0,      "kd_att_y": 0.0, 
                                    "kp_vel_xy": 1.0,     "ki_vel_xy": 0.0,     "kd_vel_xy": 0.2}
        
        # Good gains
        gains = {
                "off_alt": 55.26,   "kp_alt": 5.0,        "ki_alt": 0.05,       "kd_alt": 4.0,
                                    "kp_att_rp": 1.0,     "ki_att_rp":0.0,      "kd_att_rp": 0.35,
                                    "kp_att_y": 1.5,      "ki_att_y": 0.0,      "kd_att_y": 0.05, 
                                    "kp_vel_xy": 1.8,     "ki_vel_xy": 0.0,     "kd_vel_xy": 0.32}
        
        # Reference clipping (only for advanced users)
        max_attitude = 0.5 #[rad]
        max_yawrate = 2.0 #[rad/s]
        max_vel = 1 #[m/s]

        # Command clipping (only for advanced users)
        max_command_attitude = 1 #[]
        max_command_altitude = 10 #[]

        # Actions
        desired_vx, desired_vy, desired_yaw_rate, desired_alt = action[0], action[1], action[2], action[3]

        if self.tuning_level != "off":
            desired_vx, desired_vy, desired_yaw_rate, desired_alt = 0,0,0,1

        # Tuning velocity PID
        if self.tuning_level == "velocity":
            desired_vx = self.tuning(-max_vel,max_vel,3,dt,desired_vx, actual_vx, "velocity [m/s]")

        # Velocity PID control
        desired_vx = np.clip(desired_vx, -max_vel, max_vel)
        desired_vy = np.clip(desired_vy, -max_vel, max_vel)

        vxError = desired_vx - actual_vx
        vxDeriv = (vxError - self.pastVxError) / dt
        vyError = desired_vy - actual_vy
        vyDeriv = (vyError - self.pastVyError) / dt
        self.intVx += vxError * dt
        self.intVy += vyError * dt
        self.intVx = np.clip(self.intVx,-max_vel/2,max_vel/2)
        self.intVy = np.clip(self.intVy,-max_vel/2,max_vel/2)
        desired_pitch = gains["kp_vel_xy"] * vxError + gains["kd_vel_xy"] * vxDeriv + gains["ki_vel_xy"] * self.intVx
        desired_roll = -gains["kp_vel_xy"] * vyError - gains["kd_vel_xy"] * vyDeriv + gains["ki_vel_xy"] * self.intVy
        self.pastVxError = vxError
        self.pastVyError = vyError

        # Tuning altitude PID
        if self.tuning_level == "altitude":
            desired_alt = self.tuning(0.5,1.5,10,dt,desired_alt, actual_alt, "altitude [m]")

        # Altitude PID control
        altError = desired_alt - actual_alt
        altDeriv = (altError - self.pastAltError) / dt
        self.intAlt += altError * dt
        self.intAlt = np.clip(self.intAlt,-2,2)
        altCommand = gains["kp_alt"] * altError + gains["kd_alt"] * altDeriv + gains["ki_alt"] * self.intAlt
        self.pastAltError = altError

        # Tuning attitude PID
        if self.tuning_level == "attitude":
            desired_pitch = self.tuning(-max_attitude,max_attitude,2.0,dt,desired_pitch, actual_pitch, "attitude [rad]")
        elif self.tuning_level == "yawrate":
            desired_yaw_rate = self.tuning(-max_yawrate,max_yawrate,2.0,dt,desired_yaw_rate, actual_yaw_rate, "yawrate [rad/s]")

        # Attitude PID control
        desired_pitch = np.clip(desired_pitch, -max_attitude, max_attitude)
        desired_roll = np.clip(desired_roll, -max_attitude, max_attitude)
        desired_yaw_rate = np.clip(desired_yaw_rate, -max_yawrate, max_yawrate)

        pitchError = desired_pitch - actual_pitch
        pitchDeriv = (pitchError - self.pastPitchError) / dt
        rollError = desired_roll - actual_roll
        rollDeriv = (rollError - self.pastRollError) / dt
        yawRateError = desired_yaw_rate - actual_yaw_rate
        yawRateDeriv = (yawRateError - self.pastYawrateError) / dt
        self.intPitch += pitchError * dt
        self.intRoll += rollError * dt
        self.intYawrate += yawRateError * dt
        self.intPitch = np.clip(self.intPitch,-max_attitude/2,max_attitude/2)
        self.intRoll = np.clip(self.intRoll,-max_attitude/2,max_attitude/2)
        self.intYawrate = np.clip(self.intYawrate,-max_yawrate/2,max_yawrate/2)
        rollCommand = gains["kp_att_rp"] * rollError + gains["kd_att_rp"] * rollDeriv + gains["ki_att_rp"] * self.intPitch
        pitchCommand = -gains["kp_att_rp"] * pitchError - gains["kd_att_rp"] * pitchDeriv + gains["ki_att_rp"] * self.intRoll
        yawCommand = gains["kp_att_y"] * yawRateError + gains["kd_att_y"] * yawRateDeriv + gains["ki_att_y"] * self.intYawrate
        self.pastPitchError = pitchError
        self.pastRollError = rollError
        self.pastYawrateError = yawRateError
        
        altCommand = np.clip(altCommand,-max_command_altitude,max_command_altitude) + gains["off_alt"]
        rollCommand = np.clip(rollCommand,-max_command_attitude,max_command_attitude)
        pitchCommand = np.clip(pitchCommand,-max_command_attitude,max_command_attitude)
        yawCommand = np.clip(yawCommand,-max_command_attitude,max_command_attitude)

        # Motor mixing
        m1 =  altCommand - rollCommand + pitchCommand + yawCommand
        m2 =  altCommand - rollCommand - pitchCommand - yawCommand
        m3 =  altCommand + rollCommand - pitchCommand + yawCommand
        m4 =  altCommand + rollCommand + pitchCommand - yawCommand

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        self.global_time += dt
        return [m1, m2, m3, m4]
    
    # Only for tuning
    def set_tuning(self,level):
        self.tuning_level = level

    def tuning(self,input_min,input_max,T,dt,desired,actual,ylabel):
        if self.global_time > self.tuning_start:
            self.tuning_on = True
        if self.tuning_on:
            if (self.tuning_iter > 0):
                desired = self.step_function(dt,input_min,input_max,T)

                self.tuning_desired.append(desired)
                self.tuning_actual.append(actual)
                self.tuning_ts.append(self.global_time)
            else:
                self.plot(ylabel)
                self.tuning_on = False
                self.tuning_start = np.inf
        return desired

    def step_function(self,dt,input_min,input_max,T):
        # Calculate step function
        if self.tuning_time < T:
            input = input_min
        else:
            input = input_max
        
        # Keep track of cycle
        self.tuning_time += dt
        if self.tuning_time >= 2*T:
            self.tuning_time = 0
            self.tuning_iter -= 1
        return input
    
    def plot(self,ylabel):
        c_actual = "black"
        c_desired = "grey"
        c_os = [0/255,109/255,143/255]
        c_ss = [181/255,78/255,44/255]
        c_rt = [196/255,158/255,69/255]

        fig,ax = plt.subplots(1,1,figsize=(7,5))
        ax.plot(self.tuning_ts,self.tuning_desired,label="desired",color=c_desired)
        ax.plot(self.tuning_ts,self.tuning_actual,label="actual",color=c_actual)

        # Calculate steadt state error
        des_min = np.min(self.tuning_desired)
        des_max = np.max(self.tuning_desired)
        std = (des_max - des_min)/2

        idx_ss_high = -1
        desired_reverse = self.tuning_desired[::-1]
        idx_ss_low = len(self.tuning_ts) - np.argmin(np.gradient(desired_reverse)) - 2
        perc_min = (self.tuning_actual[idx_ss_low]-self.tuning_desired[idx_ss_low])/std*100
        perc_max = (self.tuning_actual[idx_ss_high]-self.tuning_desired[idx_ss_high])/std*100
        if (abs(perc_max) >= 1):
            ax.plot([self.tuning_ts[idx_ss_high],self.tuning_ts[idx_ss_high]],[self.tuning_desired[idx_ss_high],self.tuning_actual[idx_ss_high]],
                    linewidth=3,color=c_ss,label="steady state error")
            ax.text(x=self.tuning_ts[idx_ss_high],y=self.tuning_actual[idx_ss_high],s=str(int(abs(perc_max)))+" [%]",
                    color=c_ss,fontsize="x-large",horizontalalignment="right",verticalalignment="center")
        if (abs(perc_min) >= 1):
            ax.plot([self.tuning_ts[idx_ss_low],self.tuning_ts[idx_ss_low]],[self.tuning_desired[idx_ss_low],self.tuning_actual[idx_ss_low]],
                    linewidth=3,color=c_ss)
            ax.text(x=self.tuning_ts[idx_ss_low],y=self.tuning_actual[idx_ss_low],s=str(int(abs(perc_min)))+" [%]",
                    color=c_ss,fontsize="x-large",horizontalalignment="right",verticalalignment="center")
        
        idx_low = len(self.tuning_actual) - 2*(len(self.tuning_actual)-idx_ss_low)

        # Calculate overshoot
        actual_cut = self.tuning_actual[idx_ss_low:-1]
        idx_os_high = idx_ss_low + np.argmax(actual_cut)
        actual_cut = self.tuning_actual[idx_low:idx_ss_low]
        idx_os_low = idx_low + np.argmin(actual_cut)
        perc_max = (self.tuning_actual[idx_os_high]-self.tuning_desired[idx_os_high])/std*100
        perc_min = (self.tuning_actual[idx_os_low]-self.tuning_desired[idx_os_low])/std*100
        if (self.tuning_desired[idx_os_high] < self.tuning_actual[idx_os_high]) & (abs(perc_max) >= 1):
            ax.plot([self.tuning_ts[idx_os_high],self.tuning_ts[idx_os_high]],[self.tuning_desired[idx_os_high],self.tuning_actual[idx_os_high]],
                    linewidth=3,color=c_os,label="overshoot")
            ax.text(x=self.tuning_ts[idx_os_high],y=self.tuning_actual[idx_os_high],s=str(int(abs(perc_max)))+" [%]",
                    color=c_os,fontsize="x-large",horizontalalignment="right",verticalalignment="center")
        if (self.tuning_desired[idx_os_low] > self.tuning_actual[idx_os_low]) & (abs(perc_max) >= 1):
            ax.plot([self.tuning_ts[idx_os_low],self.tuning_ts[idx_os_low]],[self.tuning_desired[idx_os_low],self.tuning_actual[idx_os_low]],
                    linewidth=3,color=c_os)
            ax.text(x=self.tuning_ts[idx_os_low],y=self.tuning_actual[idx_os_low],s=str(int(abs(perc_min)))+" [%]",
                    color=c_os,fontsize="x-large",horizontalalignment="right",verticalalignment="center")
            
        # Calculate rise time
        limit = 0.05
        actual_cut = self.tuning_actual[idx_ss_low:-1]
        idx_rt_high = idx_ss_low + np.argmax((des_max-actual_cut)/des_max < limit)
        actual_cut = self.tuning_actual[idx_low:idx_ss_low]
        idx_rt_low = idx_low + np.argmax((actual_cut-des_min)/des_min < limit)
        
        rt_high = self.tuning_ts[idx_rt_high] - self.tuning_ts[idx_ss_low]
        rt_low = self.tuning_ts[idx_rt_low] - self.tuning_ts[idx_low]
        if idx_rt_high > idx_ss_low:
            ax.plot([self.tuning_ts[idx_ss_low],self.tuning_ts[idx_rt_high]],[self.tuning_desired[idx_rt_high],self.tuning_desired[idx_rt_high]],
                    linewidth=3,color=c_rt,label="rise time")
            ax.text(x=self.tuning_ts[idx_rt_high],y=self.tuning_actual[idx_rt_high],s=str(np.round(rt_high,1))+"[s]",
                    color=c_rt,fontsize="x-large",horizontalalignment="right",verticalalignment="bottom")
        if idx_rt_low > idx_low:
            ax.plot([self.tuning_ts[idx_low],self.tuning_ts[idx_rt_low]],[self.tuning_desired[idx_rt_low],self.tuning_desired[idx_rt_low]],
                    linewidth=3,color=c_rt)
            ax.text(x=self.tuning_ts[idx_rt_low],y=self.tuning_actual[idx_rt_low],s=str(np.round(rt_low,1))+"[s]",
                        color=c_rt,fontsize="x-large",horizontalalignment="right",verticalalignment="bottom")

        # Plot the smoothed version of tuning_actual that is used to detect the overshoot
        # ax.plot(self.tuning_ts,self.moving_average(self.tuning_actual,10),label="smoothed",color=c_rt)

        ax.set_xlabel("time [s]")
        ax.set_ylabel(ylabel)
        plt.suptitle("PID tuning")
        plt.legend(loc="upper left")
        plt.tight_layout()
        plt.show()

        self.tuning_level = "off"

    # def moving_average(self,data,window_size):
    #     # Define the kernel for the moving average
    #     kernel = np.ones(window_size) / window_size

    #     # Use 'same' mode to ensure the output has the same length as the input
    #     smoothed_data = np.convolve(data, kernel, mode='same')
    #     return smoothed_data