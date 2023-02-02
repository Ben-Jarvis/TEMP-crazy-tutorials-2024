Project description
===================

In this project, all students have to perform the following task in the simulator based on `Webots <https://cyberbotics.com/>`_,
which provides the same sensor readings and control outputs as those in the real world.
The leaderboard page will display the simulation performance of all students' algorithm based on the uploaded code.
In the last three weeks, all students will compete as groups to achieve the same task on the real drone.

Task overview
-------------

.. image:: objective_figure.png
  :width: 650
  :alt: objective figure

In this practical, you will learn how to program a Crazyflie to find and precisely land on a platform with the help of minimal sensory information.
Additionally, you will use sensor readings to avoid the obstacles present in the environment.
After finding and landing on the landing pad, the drone needs to fly back and land on the take-off pad.

Performance metrics
-------------------

- **Grade 4.0**: Take off and avoid obstacles
- **Grade 4.5**: Find the landing pad and land on it
- **Grade 5.0**: Take off again and explore again
- **Grade 5.5**: Find and land on the take-off pad
- **Grade 5.5 + 0.5 * (1 - (t_i – min(t)) / (max(t) – min(t)) )**: Based on the flight time relative to the other completed runs (the faster, the better)
- **final_grade = 0.7 * sim_grade + 0.3 * hardware_grade**: Simulation and hardware task are graded after the same metrics. The final grade is a weighted average of the two

For more details on the task, submission, schedule and grading, please refer to the `moodle <https://moodle.epfl.ch/course/view.php?id=15799>`_ page of the course.

Sim2real transfer
-----------------
The data flow diagram for both simulation and real quadrotor is shown as below.
Since they have same types of sensory inputs and control outputs, your algorithm in simulation can be tuned slightly for real-drone control.

.. image:: sim2real.svg
  :width: 650
  :alt: sim2real