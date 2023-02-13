Examples
========

Here are some basic algorithms which may be helpful to drone exploration.

Obstacle avoidance
------------------
This is a simple obstacle avoidance algorithm based on the distance sensors.
For example, if the front distance is less than 0.2m, the drone will move left when left distance is larger than the right distance.
The drone will move right when right distance is larger than the left distance.
However, if the front distance is larger than 0.2m, the drone will move forward.
The code for this algorithm is::

    on_ground = True
    height_desired = 0.0
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

This example can be found in 'example.py' file.
You can run this example by uncommenting 'control_commands = example.obstacle_avoidance(sensor_data)' in the 'main.py' file.
Simulation results of this algorithm is shown in the following animation.

.. image:: example_obstacle_avoidance.gif
  :width: 650
  :alt: demo video from last year

Basic exploration
-----------------
Search all area (e.g., line search) to find the landing box.

Create a world map
------------------
This is a simple occupancy grid algorithm based on distance measurements.
We loop through all four distance sensors, filling the occupancy map for each of them based on the following rules:
If an obstacle is detected, a conficdence value is subtracted, or added in case of a free cell.
If we don't have any information about that cell, it stays as it is.
At the end of the loop, we normalize the occupancy map to be between -1 and 1.
This resembles confidence about the occupancy of each cell and is more robust than just always filling in the last measurement.

The code for this algorithm is::
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the point is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
    t +=1

    return map

This example can be found in 'example.py' file.
You can run this example by uncommenting 'map = example.occupancy_map(sensor_data)' in the 'main.py' file.
Simulation results of this algorithm is shown in the following animation.

.. image:: example_occupancy_map.gif
  :width: 650
  :alt: occupancy map example