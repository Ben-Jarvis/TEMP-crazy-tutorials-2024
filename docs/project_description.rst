Project description
===================

In this project, all students have to perform the following task in the simulator based on `Webots <https://cyberbotics.com/>`_,
which provides the same sensor readings and control outputs as those in the real world.
The leaderboard page will display the real-time performance of all students' algorithm based on the uploaded code.
In the last two weeks, the top 20 students in the leaderboard will have chance to put their method on the real drone.

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

- Take off and avoid obstacles: 4
- Find the landing pad and land on it: 4.5
- Take off again and explore again: 5
- Find and land on the take-off pad: 5.5
- Top 20 with the fastest accomplishment time will do the real-world test: 5.75 (50%) and 6.0 (50%) according to the finishing time

For more details on the task, submission, schedule and grading, please refer to the `moodle <https://moodle.epfl.ch/course/view.php?id=15799>`_ page of the course.

Sim2real transfer
-----------------
The system schemes for both simulation and real quadrotor are shown as below.
Since they have same types of sensory inputs and control outputs, your algorithm in simulation can be tuned slightly and used for real drone control.

.. image:: sim2real.svg
  :width: 650
  :alt: sim2real