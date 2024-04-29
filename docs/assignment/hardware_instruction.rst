Hardware instruction
====================

You should be familiar with the project task through simulation.
As a group, you can now fine-tune the best solution to control a real quadrotor in cluttered environments. 
Compared to the simulation, you will find that you may have to adjust your algorithms so that they can be executed in real-time on the hardware. 
You should also keep in mind that the sensor signals on the actual drone are noisy and less accurate than in simulation.
You will lastly present your algorithm and demonstrate its performance live on the real Crazyflie at the end of the course.

Learning objectives
-------------------
- Controlling a plug-and-play drone
- Interfacing with the Python library to control the drone
- Learning the difference between code deployment in simulation and on the real drone
- Mastering different flight phases in the real world
- Reporting performance results in a scientific manner

Final demonstration
-------------------
The final tasks which you will be graded on for this hardware practical are the following:
- Short presentation (4 slides max)
- Video recording of one of your trials in an obstacle-rich environment
- Demonstration on May 28th: 3 trials with obstacles (**best grade counts**), and each trial must be completed with only one battery

Presentation
------------
- max 4 slides, max 7 min
- 1 slide on the experimental setup (environment layout)
- 1 slide on the strategy (algorithm, what you spend most time on)
- 1 slide on the results (statistics on mission time/success/...)
- 1 optional slide (with anything relevant to add)

Video of one trial
------------------
- be max 2 min long
- contain no edits except speed-ups (indicate speed up factor) and text additions
- show at least one of the team members
- show a clock/stop watch (either with physical device or with video editing) that indicates seconds at the beginning and at the end
- be in mp4 format
- You can create your own obstacles, for example checking the last-year video in :ref:`Task overview`

Complementary information
-------------------------
- **Submit by Monday 29th May, 23:59** (your code with comments, the video as MP4, presentation as PPTX) in a zip file named GROUPNUMBER_LASTNAME1_LASTNAME2_LASTNAME3_LASTNAME4_LASTNAME5.zip on moodle
- To get sufficiently good position and velocity estimates from the optic flow sensor, the ground below the crazyflie should contain some texture
- Initial position of the drone will be given one day before the final exam
- Each group will present the PPTX from our computer before the demonstration
- Return the drone directly after the demonstration
- Check that the returned material is complete, according to the lists in :ref:`Hardware unpacking`
- Make sure you set up and make space quickly

Hardware grading
----------------
- presentation and video of working algorithm, 0-4
- takeoff, avoid obstacles in the way, 4-4.5
- locate and land on the landing pad stably, 4.5-5
- go back, land on take-off pad, 5-5.5
- time grading for completing the full task: 5.5-6 (Top 25% get 6, middle 50% get 5.75, final 25% get 5.5)

In case of a clear gap between flight performance at the exam vs in the video, we will account for the performance in the video you sent.

Demonstration schedule on May 30
--------------------------------
============= ============= ==============
Time          Room MED11518 Room MED-11422
============= ============= ==============
15:15 - 15:30 group 1       group 9
15:35 - 15:50 group 2       group 10
15:55 - 16:10 group 3       group 11
16:15 - 16:30 group 7       group 12
16:35 - 16:50 group 5       group 13
16:55 - 17:10 group 6       group 14
17:15 - 17:30 group 4       group 15
17:35 - 17:50 group 8       group 16
17:55 - 18:10               group 17
============= ============= ==============
