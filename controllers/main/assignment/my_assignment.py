import numpy as np
import time
import cv2

# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py lines 296-323. 
# The "item" values that you can later use in the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "z_global": Global Z position
# "range_down": Downward range finder distance
# "range_front": Front range finder distance
# "range_left": Leftward range finder distance 
# "range_right": Rightward range finder distance
# "range_back": Backward range finder distance
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)

def get_command(sensor_data, camera_data, dt):

    # Open a window to display the camera image
    # NOTE: Displaying the camera image will slow down the simulation, this is just for testing
    # cv2.imshow('Camera Feed', camera_data)
    # cv2.waitKey(1)
    
    time.sleep(0.5)

    # Take off
    if sensor_data['z_global'] < 0.49:
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
    
    return control_command # Ordered as array with: [v_forward_cmd, v_left_cmd, alt_cmd, yaw_rate_cmd]