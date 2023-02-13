# Examples of basic methods for simulation competition
import numpy as np

on_ground = True
height_desired = 0.0

# Obstacle avoidance with range sensors
def obstacle_avoidance(sensor_data):
    global on_ground, height_desired

    # Take off with incremental height until height is above 0.5
    if on_ground and sensor_data['altitude'] < 0.5:
        height_desired += 0.01
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Obstacle avoidance with distance sensors
    if sensor_data['range_front'] < 0.2:
        if sensor_data['range_left'] > sensor_data['range_right']:
            control_command = [0.0, 0.2, 0.0, height_desired]
        else:
            control_command = [0.0, -0.2, 0.0, height_desired]
    else:
        control_command = [0.2, 0.0, 0.0, height_desired]

    return control_command

# Grid based coverage path planning
setpoints = [[1.0, 2.0], [2.0, -2,0]]
index_current_setpoint = 0
def path_planning_with_grid(sensor_data):
    global on_ground, height_desired

    # Take off with incremental height until height is above 0.5
    if on_ground and sensor_data['altitude'] < 0.5:
        height_desired += 0.01
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False
    
    # Calculate the control command based on current goal setpoint
    x_goal, y_goal = setpoints[index_current_setpoint]
    x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
    