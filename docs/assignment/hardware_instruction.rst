Hardware instructions
=====================

You should already be familiar with the project task through your work in simulation.
As a group, you will now fine-tune the best solution to fly a real quadrotor through gates. 
You should keep in mind that the sensor signals on the actual drone are noisy and less accurate than in simulation, we recommend starting slow. 
You may furthermore tune your PID control gains in the Crazyflie framework to achieve better performance.

Learning objectives
-------------------
- Controlling a plug-and-play drone
- Interfacing with the Python library to control the drone
- Learning the difference between code deployment in simulation and on the real drone
- Mastering different flight phases in the real world
- Reporting performance results in a scientific manner

Tasks
-----
The final deliverables which you will be graded on for this hardware practical are the following:

- Short presentation (4 slides maximum)
- Video recording of your most successful trials (up to 2 minutes)
- Task video, code and presentation files submitted on Moodle by May 26th 23:59
- Demonstration on May 27th: 3 trials (**best grade counts**), and each trial must be completed with only one battery

Presentation
------------
Length: up to 4 slides

Duration: up to 7 minutes

- 1 slide on the gate setup for your experiment (Show environment layout(s) tested on)
- 1 slide on the strategy (Algorithm, what you spend most time on)
- 1 slide on the results (Statistics on mission time/success/...)
- 1 optional slide (with anything relevant to add)

Video of one trial
------------------
You will prepare and submit a video of one of your trials with your own obsatcles. The video should:

- Be at most 2 minutes long
- Contain no edits except speed-ups (indicate speed up factor in overlay) and text additions
- Show at least one of the team members
- Show a clock/stop watch with a physical device (i.e a timer on a phone) that indicates seconds at the beginning and at the end
- Be in mp4 format

Demonstrations
--------------

The demonstrations and presentations will take place at assigned timeslots on May 27th, between 15:15 and 20:00 (slot assignment will be provided later).

Experiments and development
----------------------------

To develop and test your algorithms, you will use our pre-built setup with the Lighthouse positioning system in the MED-1 1422 drone arena as shown in the figure below.

.. image:: hardware_setup.png
  :width: 650
  :alt: hardware course

Below is an example "safe" (so unoptimized) one-lap test-run of the course, which has to be conducted counter-clockwise. 

.. image:: hardware_testrun.gif
  :width: 650
  :alt: hardware course run

The setup consists of 4 gates. The drone must take-off from and land from on the indicated landing pad after your three laps.
You may reposition the gates to your liking during your own testing, whilst please maintaining a good working order and placing them back neatly within the testing space.

In this `form <https://docs.google.com/spreadsheets/d/1jxJD-PnUoYsJz4ouRZlyiNg_vVRUKn35aWl69dxFwjU/edit?gid=1016483530#gid=1016483530>`_, each group will be able to book slots for use of the setup for a maximum of 3 hours per day up to the deadline, which you can split up as you like. Bookings can be made at maximum two workday weeks in advance.

A representative group captain must provide their name, surname, SCIPER and group number by e-mail to charbel.toumieh@epfl.ch to obtain drone arena access.

Complementary information
-------------------------

- **Submit by Monday 26th May, 23:59** (your code with comments, the video as MP4, presentation as PPTX) in a zip file named GROUPNUMBER_LASTNAME1_LASTNAME2_LASTNAME3_LASTNAME4_LASTNAME5.zip on Moodle
- The initial position of the landing pad and the location of the gates will be given one day before the final demonstration
- Each group will present the PPTX from our computer before the demonstration
- Return the drone directly after the demonstration
- Check that the returned material is complete, according to the lists in :ref:`Hardware unpacking`
- Make sure you set up and make space quickly

Overall grading for the hardware task
-------------------------------------

We will evaluate you as a group and determine your grade for this hardware task as a weighted average of the following elements:

- Presentation and video showcasing the algorithm on one of your trials (Grade 1-6, Weighting: 50%) 
- Demonstration (Grade 1-6, Weighting: 50%)

The assistants will judge your presentation right before your hardware demonstration. Your video submission to moodle will be judged separately in the same week.

The demonstration performance will be graded according to the performance metrics defined under :ref:`Hardware Task overview` and are the same as in simulation:

- **Grade 3.5**: Take off
- **Grade 3.5 - 4.75**: For each gate passed through in the first lap you get + 0.25
- **Grade 4.75 - 6.0**: Average time over the second and third lap compared to the rest of the class

.. Demonstration schedule on May 28th
.. ----------------------------------
.. ============= ============= ==============
.. Time          Room MED11518 Room MED-11422
.. ============= ============= ==============
.. 15:15 - 15:30 group 1       group 11
.. 15:35 - 15:50 group 2       group 12
.. 15:55 - 16:10 group 3       group 13
.. 16:15 - 16:30 group 4       group 14
.. 16:35 - 16:50 group 5       group 15
.. 16:55 - 17:10 group 6       group 16
.. 17:15 - 17:30 group 7       group 17
.. 17:35 - 17:50 group 8       group 18
.. 17:55 - 18:10 group 9       group 19
.. 18:15 - 18:30 group 10      group 20
.. ============= ============= ==============
