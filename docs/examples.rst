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
You can run this example by uncommenting 'control_commands = example.obstacle_avoidance(sensor_data)' in 'main.py' file.
Simulation results of this algorithm is shown in the following animation.

.. image:: example_obstacle_avoidance.gif
  :width: 650
  :alt: demo video from last year

Basic exploration
-----------------
Search all area (e.g., line search) to find the landing box.

Create a world map
------------------
