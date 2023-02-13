# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.0

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):

        # Take off with incremental height until height is above 0.5
        if self.on_ground and sensor_data['altitude'] < 0.5:
            self.height_desired += 0.01
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        # Maintain the desired height when the drone is in the air
        else:
            control_command = [0.0, 0.0, 0.0, self.height_desired] 
            on_ground = False
            return control_command