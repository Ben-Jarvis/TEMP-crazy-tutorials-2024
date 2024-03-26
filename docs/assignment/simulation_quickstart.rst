Simulation quickstart
=====================

Choose world
--------------
Make sure you load **crazyflie_world_assignment.wbt** in Webots.

Remember to select 'Close without Saving' when you close Webots software, such that you will not change the simulated world environments.

Controller switch
-----------------
In **main.py**, **choose exp_num = 3** for this assignment. 

Switch **control_syle = 'keyboard'** to fly on your own (W: forward, S: backward, A: left, D: right, Q: yaw left, E: yaw right, C: lower altitude, V: higher altitude). 

Switch **control_syle = 'path_planner'** to choose automatic control. You can edit your own algorithm in **my_control.py**, **mission_planner()**.

All you need to do is to create your algorithms in the **my_control.py** file which you should submit for getting simulation grades. Details on how to submit can be found in the section “Leaderboard”.