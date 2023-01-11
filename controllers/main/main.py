# Simulation with OpenAI gym style (example of PPO reinforcement learning)
# Author: Shushuai Li at EPFL LIS Lab
# Based on openai_gym.py in webots and crazyflie_controller_py.py
# To use conda, run 'webots worlds/crazyflie_world_epfl_lis.wbt'

import sys
from controller import Supervisor, Keyboard
from pid_control import pid_velocity_fixed_height_controller
from math import cos, sin

try:
    import gym
    import numpy as np
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_checker import check_env
    gym.logger.set_level(40) # avoid warning
except ImportError:
    sys.exit(
        'Please make sure you have all dependencies installed. '
        'Run: "pip3 install numpy gym stable_baselines3"'
    )

# Create gym environment in webots
class CrazyflieInDroneDome(Supervisor, gym.Env):
    def __init__(self):
        super().__init__()
        # Action space: [forward_velocity, left_velocity, yaw_rate]
        self.action_space = gym.spaces.Box(low = np.array([-2.0, -2.0, -1.0]),
                                        high = np.array([2.0, 2.0, 1.0]),
                                        dtype = np.float32)

        # Observation space: [pos_x_to_goal, pos_y_to_goal, range_front,
        #                     range_left, range_back, range_right]
        self.observation_space = gym.spaces.Box(
                                    low = np.array([-20.0, -20.0, 0.0, 0.0, 0.0, 0.0]),
                                    high = np.array([20.0, 20.0, 2.0, 2.0, 2.0, 2.0]),
                                    dtype = np.float32)

        # Environment specific
        self.timestep = int(self.getBasicTimeStep())

        # Actuators
        self.m1_motor = []
        self.m2_motor = []
        self.m3_motor = []
        self.m4_motor = []

        # Sensors
        self.imu = []
        self.gps = []
        self.gyro = []
        self.camera = []
        self.range_finder = []
        self.range_front = []
        self.range_left = []
        self.range_back = []
        self.range_right = []

        # History variables
        self.pastXGlobal = 0
        self.pastYGlobal = 0
        self.past_time = self.getTime()

        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_time = 0

        # Task variables
        self.goal_position_x = 2.0
        self.goal_position_y = 0.0
        self.step_count = 0

        # Tools
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)

    def wait_keyboard(self):
        while self.keyboard.getKey() != ord('Y'):
            super().step(self.timestep)

    def action_from_keyboard(self):
        forwardVelocityDesired = 0
        leftVelocityDesired = 0
        yawRateDesired = 0
        key = self.keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forwardVelocityDesired = 1.0
            elif key == Keyboard.DOWN:
                forwardVelocityDesired = -1.0
            elif key == Keyboard.RIGHT:
                leftVelocityDesired = -1.0
            elif key == Keyboard.LEFT:
                leftVelocityDesired = 1.0
            elif key == ord('Q'):
                yawRateDesired = 1.0
            elif key == ord('E'):
                yawRateDesired = -1.0
            key = self.keyboard.getKey()
        return [forwardVelocityDesired, leftVelocityDesired, yawRateDesired]

    def get_observations(self):
        # Get robot position
        xGlobal = self.gps.getValues()[0]
        yGlobal = self.gps.getValues()[1]

        # Get robot yaw direction
        actualYaw = self.imu.getRollPitchYaw()[2]
        cosyaw, sinyaw = cos(actualYaw), sin(actualYaw)

        # Get relative position to the goal
        pos_x_to_goal = self.goal_position_x - xGlobal
        pos_y_to_goal = self.goal_position_y - yGlobal
        pos_x_to_goal_bodyframe = pos_x_to_goal * cosyaw + pos_y_to_goal * sinyaw
        pos_y_to_goal_bodyframe = -pos_x_to_goal * sinyaw + pos_y_to_goal * cosyaw

        # Range sensor measurements
        range_front = self.range_front.getValue() / 1000.0
        range_left = self.range_left.getValue() / 1000.0
        range_back = self.range_back.getValue() / 1000.0
        range_right = self.range_right.getValue() / 1000.0

        return np.array([pos_x_to_goal_bodyframe, pos_y_to_goal_bodyframe,
            range_front, range_left, range_back, range_right]).astype(np.float32)

    def reset(self):
        ## Reset the simulation
        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.timestep)

        # Reser motors
        self.m1_motor = self.getDevice("m1_motor");
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.getDevice("m2_motor");
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.getDevice("m3_motor");
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.getDevice("m4_motor");
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        # Reset sensors
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

        # Reset PID
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_time = 0
        self.past_time = self.getTime()
        self.step_count = 0

        # Internals
        super().step(self.timestep)

        # Take off with pure PID
        for time in range(150):
            self.step([0, 0, 0])
        self.step_count = 0

        # Open AI Gym generic
        return self.get_observations()

    def step(self, action):
        dt = self.getTime() - self.past_time
        # Get measurements
        actual_roll = self.imu.getRollPitchYaw()[0]
        actual_pitch = self.imu.getRollPitchYaw()[1]
        actual_yaw_rate = self.gyro.getValues()[2]
        actual_alt = self.gps.getValues()[2]
        xGlobal = self.gps.getValues()[0]
        vxGlobal = (xGlobal - self.pastXGlobal) / dt
        yGlobal = self.gps.getValues()[1]
        vyGlobal = (yGlobal - self.pastYGlobal) / dt

        # Get body fixed velocities
        actual_yaw = self.imu.getRollPitchYaw()[2]
        cosyaw = cos(actual_yaw)
        sinyaw = sin(actual_yaw)
        actual_vx =  vxGlobal * cosyaw + vyGlobal * sinyaw
        actual_vy = -vxGlobal * sinyaw + vyGlobal * cosyaw

        # Low-level PID velocity control with fixed height
        desired_alt = 1.0
        motorPower = self.PID_CF.pid(dt, action, desired_alt, actual_roll, actual_pitch,
                                    actual_yaw_rate, actual_alt, actual_vx, actual_vy)
        
        # Update motor command with lower frequency than sensor update
        # self.PID_update_time += 1
        # if self.PID_update_time == 3:
        #     self.PID_update_time = 0
        self.m1_motor.setVelocity(-motorPower[0])
        self.m2_motor.setVelocity(motorPower[1])
        self.m3_motor.setVelocity(-motorPower[2])
        self.m4_motor.setVelocity(motorPower[3])

        self.past_time = self.getTime()
        self.pastXGlobal = xGlobal
        self.pastYGlobal = yGlobal

        super().step(self.timestep)

        # Observation
        observation = self.get_observations()

        # Range sensor measurements
        range_front = self.range_front.getValue() / 1000.0
        range_left = self.range_left.getValue() / 1000.0
        range_back = self.range_back.getValue() / 1000.0
        range_right = self.range_right.getValue() / 1000.0
        done = bool(range_front < 0.1 or range_left < 0.1 or
                    range_back < 0.1 or range_right < 0.1)

        # Reward
        reward = -np.linalg.norm(observation[0:2])

        self.step_count += 1
        if self.step_count > 300:
            done = True

        # Open AI Gym generic
        return observation, reward, done, {}

def main():
    # Initialize the environment
    env = CrazyflieInDroneDome()
    # check_env(env)

    # Train with reinforcement learning
    # model = PPO('MlpPolicy', env, verbose=1)
    # model.learn(total_timesteps=1e2)
    # model.save("ppo_crazyflie")
    # model.load("ppo_crazyflie")

    # Replay
    # print('Training is finished, press `Y` for replay...')
    # env.wait_keyboard()

    observation = env.reset()
    for _ in range(100000):

        # Action from trained PPO
        # action, _states = model.predict(observation)

        # Action from keyboard
        action = env.action_from_keyboard()

        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()

if __name__ == '__main__':
    main()