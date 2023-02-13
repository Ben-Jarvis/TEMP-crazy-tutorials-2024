Project description
===================

In this project, all students have to perform the following task in the simulator based on `Webots <https://cyberbotics.com/>`_,
which provides the same sensor readings and control outputs as those in the real world.
The leaderboard page will display the simulation performance of all students' algorithms based on the uploaded code.
In the last three weeks, all students will compete as groups to achieve the same task on the real drone.

Task overview
-------------

.. image:: objective_figure.png
  :width: 650
  :alt: objective figure

In this practical, you will learn how to program a Crazyflie to find and precisely land on a platform with the help of minimal sensory information.
Additionally, you will use sensor readings to avoid the obstacles present in the environment.
After finding and landing on the landing pad, the drone needs to fly back and land on the take-off pad.

System scheme
-------------
The data flow diagram for both the simulation and the real quadrotor is shown below.
Though they have the same types of sensory inputs and control outputs, your algorithm in simulation should be tuned in order to control the real drone.

.. image:: sim2real.svg
  :width: 650
  :alt: sim2real

Performance metrics
-------------------

- **Grade 4.0**: Take off and avoid obstacles
- **Grade 4.5**: Find the landing pad and land on it
- **Grade 5.0**: Take off again and explore again
- **Grade 5.5**: Find and land on the take-off pad
- **Grade 5.5 + 0.5 * (1 - (t_i – min(t)) / (max(t) – min(t)) )**: Based on the flight time relative to the other completed runs (the faster, the better)
- **Final_grade = 0.7 * sim_grade + 0.3 * hardware_grade**: Simulation and hardware tasks are graded based on the same metrics as shown above. The final grade is a weighted average of the simulation and hardware grades.

Project schedule
----------------
The following table provides the arrangement of the crazy-practical project.

==========================  =======================================================
**Week**                    **Notes**
| Week 8, April 18          | Project introduction, Simulation installation, Q&A
| Week 9, April 25          | VIO example in Webots, Simulation development, Q&A
| Week 10, May 2            | Simulation development, Q&A
                            | Select the hardware group in Moodle before May 9
| Week 11, May 9            | Hardware introduction, drone software installation
                            | Simulation due 23:59 CET, submit the code in Moodle
| Week 12, May 16           | Hardware development, Q&A
| Week 13, May 23           | Hardware development, Q&A
| Week 14, May 30           | Hardware demonstrations, hand in the drones
==========================  =======================================================

For more details on the task, submission, schedule and grading, please refer to the `moodle <https://moodle.epfl.ch/course/view.php?id=15799>`_ page of the course.