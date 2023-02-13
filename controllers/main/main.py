# Main simulation file called by the Webots

import numpy as np
from controller import Supervisor, Keyboard
from pid_control import pid_velocity_fixed_height_controller
from my_control import MyController
import example

# Crazyflie drone class in webots
class CrazyflieInDroneDome(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Actuators
        self.m1_motor = self.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        # Sensors
        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timestep)
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timestep)
        self.range_front = self.getDevice('range_front')
        self.range_front.enable(self.timestep)
        self.range_left = self.getDevice("range_left")
        self.range_left.enable(self.timestep)
        self.range_back = self.getDevice("range_back")
        self.range_back.enable(self.timestep)
        self.range_right = self.getDevice("range_right")
        self.range_right.enable(self.timestep)

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.getTime()
        self.sensor_read_last_time = self.getTime()
        self.step_count = 0

        # History variables for calculating groundtruth velocity
        self.x_global_last = 0
        self.y_global_last = 0

        # Tools
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)

        # Simulation step update
        super().step(self.timestep)

    def wait_keyboard(self):
        while self.keyboard.getKey() != ord('Y'):
            super().step(self.timestep)

    def action_from_keyboard(self):
        forward_velocity = 0.0
        left_velocity = 0.0
        yaw_rate = 0.0
        altitude = 1.0
        key = self.keyboard.getKey()
        while key > 0:
            if key == ord('W'):
                forward_velocity = 1.0
            elif key == ord('S'):
                forward_velocity = -1.0
            elif key == ord('A'):
                left_velocity = 1.0
            elif key == ord('D'):
                left_velocity = -1.0
            elif key == ord('Q'):
                yaw_rate = 1.0
            elif key == ord('E'):
                yaw_rate = -1.0
            key = self.keyboard.getKey()
        return [forward_velocity, left_velocity, yaw_rate, altitude]

    def read_sensors(self):
        
        # Data dictionary
        data = {}

        # Time interval
        dt = self.getTime() - self.sensor_read_last_time
        self.sensor_read_last_time = self.getTime()

        # Position
        data['x_global'] = self.gps.getValues()[0]
        data['y_global'] = self.gps.getValues()[1]
        data['altitude'] = self.gps.getValues()[2]

        # Attitude
        data['roll'] = self.imu.getRollPitchYaw()[0]
        data['pitch'] = self.imu.getRollPitchYaw()[1]
        data['yaw'] = self.imu.getRollPitchYaw()[2]

        # Velocity
        vx_global = (data['x_global'] - self.x_global_last) / dt
        vy_global = (data['y_global'] - self.y_global_last) / dt
        self.x_global_last = data['x_global']
        self.y_global_last = data['y_global']
        data['v_forward'] =  vx_global * np.cos(data['yaw']) + vy_global * np.sin(data['yaw'])
        data['v_left'] =  -vx_global * np.sin(data['yaw']) + vy_global * np.cos(data['yaw'])

        # Range sensor
        data['range_front'] = self.range_front.getValue() / 1000.0
        data['range_left']  = self.range_left.getValue() / 1000.0
        data['range_back']  = self.range_back.getValue() / 1000.0
        data['range_right'] = self.range_right.getValue() / 1000.0

        # Yaw rate
        data['yaw_rate'] = self.gyro.getValues()[2]

        return data

    def reset(self):
        # Reset the simulation
        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.timestep)

    def step(self, control_commands, sensor_data):

        # Time interval for PID control
        dt = self.getTime() - self.PID_update_last_time
        self.PID_update_last_time = self.getTime()

        # Low-level PID velocity control with fixed height
        motorPower = self.PID_CF.pid(dt, control_commands, sensor_data['roll'], sensor_data['pitch'],
                                                    sensor_data['yaw_rate'], sensor_data['altitude'],
                                                    sensor_data['v_forward'], sensor_data['v_left'])
        
        # Update motor command
        self.m1_motor.setVelocity(-motorPower[0])
        self.m2_motor.setVelocity(motorPower[1])
        self.m3_motor.setVelocity(-motorPower[2])
        self.m4_motor.setVelocity(motorPower[3])

        # Update drone states in simulation
        super().step(self.timestep)


if __name__ == '__main__':

    # Initialize the drone
    drone = CrazyflieInDroneDome()
    my_controller = MyController()

    # Simulation loops
    for t in range(100000):

        # Read sensor data including []
        sensor_data = drone.read_sensors()

        # Control commands with [v_forward, v_left, yaw_rate, altitude]
        # ---- Select only one of the following control methods ---- #
        control_commands = drone.action_from_keyboard()
        # control_commands = my_controller.step_control(sensor_data)
        # control_commands = example.obstacle_avoidance(sensor_data)
        # ---- end --- #

        # Update the drone status in simulation
        drone.step(control_commands, sensor_data)

        # Grading function based on drone states and world information
        # take off and fly through obstacle area
        # land on the landing platform
        # take off and fly through again
        # land on the starting platform
        # flight time