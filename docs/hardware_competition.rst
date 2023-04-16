Hardware competition
====================

In this project, the task is to design a navigation algorithm for a quadrotor to find and land on a box, then return to the starting position while avoiding obstacles, fully relying on onboard sensors.
First, the solution can be designed and validated in a simulator based on `Webots <https://cyberbotics.com/>`_ which provides similar sensor readings and control outputs as those in the real world.
Afterwards, the solution can be fine-tuned and used to control a real crazyflie quadrotor in the physical cluttered environments.
Be aware that the sensor packages on the actual drone are not as accurate as those in the simulator, especially yawing can lead to drift in position estimation.

Crazyflie quadrotor
-------------------
- Hardware
- Software installation

Example - parameter log
-----------------------
- Obtain sensor readings from the drone to your laptop

Example - remote control
------------------------
- Send velocity control command from laptop to the drone

Fine tuning your method
-----------------------
- Adapt your method from simulation to the real drone script