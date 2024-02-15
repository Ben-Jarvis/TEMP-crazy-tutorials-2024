Exercise 2: Kalman Filtering
==================================

You have now already seen how to tune a cascaded PID controller to achieve accurate and fast waypoint flight with a quadrotor.
Up to this point, the measurement data that you have used for PID control feedback is the ground-truth (or noise-free) 
measurement that was sampled at every control step.

In reality, we are however faced with noisy sensor measurements and typically use a combination of mutliple sensors, 
each measuring different quantities with different noise levels at different time intervals, to obtain the state feedback 
for our controller.
This is where sensor fusion comes into play. As you have seen in the lecture, 
the Kalman Filter is a powerful and elegant sensor fusion technique that can both fuse measurements from different sensors 
and provide state estimates for control feedback at arbitrary time steps.
With this method, you will see that your quadrotor can stay on track in noisy, real world environments!

Task description
-----------------

In this task, you will implement a Kalman Filter in **kalman_filter.py** and explore how it can significantly improve the 
performance of a closed loop cascaded PID controller in a real-world scenario by combining sensor measurements, filtering 
measurenemt noise and providing dynamically reasonable state estimates.

Besides the Gyroscope, for this exercise, the Crazyflie drone is 
equipped with a GPS and an Accelerometer. 
These provide the following translational sensor measurements for your implementation:
- GPS: Global X,Y,Z position
- Accelerometer: Body X,Y,Z accelerations (are transformed to global frame for reference in this exercise)

Each of the sensors in the simulation and the PID controller furthermore run at the following frequencies:
- GPS: 62.5 Hz
- Accelerometer: 125 Hz
- PID Controller: 62.5 Hz

As we are considering real sensor data, the GPS and the Accelerometer measurements respectively posess a Gaussian noise with a 
certain standard deviation, defined as std_GPS = 0.2 meters and std_ACCEL = 0.01 m/s².

Let us first look at position and acceleration data obtained from these noisy measurements below:

FIGURE (Noisy position and acceleration)

To obtain the missing velocity measurements which are required for our PID controller, in a manner similar to the previous exercise, we can start by differentiating the noisy GPS position over every control interval. 
By comparing this estimate to the ground-truth velocity as shown below, we however observe that the velocity estimates are even noisier than the position measurements and far from accurate:

FIGURE (Figure of velocity noise)

When feeding these measurements directly into our cascaded PID controller, we then see the catastrophic results:

FIGURE (GIF of failed drone)

As you will see later in this exercise, a pure integration of accleration measurements to yield velocities provides similarly unsatisfactory results.

Therefore, to remedy this problem, given the noisy GPS and Acclerometer measurements and using 
the provided theory from the lecture, you will implement and tune a Kalman Filter that returns 
much better state estimates for three-dimensional position, velocity and acceleration:

FIGURE (Noise vs. KF)

Your drone should then remain in flight throughout the parcours and you can modify process 
parameters to improve the performance of your controller with the Kalman Filter!

Exercise
---------

Part 1 - Implementation
------------------------
You will begin by implementing your Kalman Filer code in **kalman_filter.py**. 
The state prediction vector is represented by a 9 x 1 column vector and must be ordered as: 
[X Position, X Velocity, X Acceleration, Y Position, Y Velocity, Y Acceleration, Z Position, Z Velocity, Z Acceleration]

Hint: For matrix multiplications with two-dimensional numpy arrays, use numpy.matmul 
or the python operator "@" 

The function **initialize_KF** initializes the Kalman Filter parameters. In this function:
1. Initialize the optimal state estimate **self.X_opt** and prediction covariance **self.P_opt**.
2. Define the sensor measurement matrices **self.H_GPS** and **self.H_ACCEL**.
3. Given the provided measurement noise standard deviations **noise_std_GPS** and **noise_std_ACCEL**, define the masurement uncertainty matrices **self.R_GPS** and **self.R_ACCEL**.

The function **KF_state_propagation** performs the propagation of the optimal states **self.X_opt** 
and optimal prediction covariance **self.P_opt** over a specified time-interval **dt**. 
In this function:
1. Given the propagation time **dt** as the function input, define the transition matrix 
**A_trans** for a particle with constant accleration as seen in the lecture.
2. As described in the lecture, update both the values of the state prediction **X_pred** and 
prediction covariance **P_pred**, given the transition matrix **A_trans**, the process uncertainty
 matrix **Q_trans** (provided in the function), the previous optimal state **self.X_opt** and the 
 optimal prediction covariance **self.P_opt**.
3. Return **X_pred** and **P_pred**.

The function **KF_sensor_fusion** performs the fusion of sensor measurements and calculates the new **self.X_opt** and **self.P_opt** once a sensor measurement is received. In this function:
1. Calculate the Kalman Filter gain **K** as seen in the lecture, given the input measurement
matrix **H**, measurement uncertainty matrix **R**, the obtained measurement **Z**, the propagated state **X_pred** and propagated covariance **P_pred**.
2. Implement the sensor fusion rule to update the new values of **self.X_opt** and **self.P_opt**.

The function **KF_estimate** returns the state estimate and prediction covariance when demanded by calling the state propagation and sensor fusion functions according to the sensor measurements obtained at the current time.
The process is akin to the situation depicted on the following slide from the lecture:

slide

In this function, a **sensor_state_flag** is provided, which can take the following values at 
every called time step: {0: No measurement received, 1: GPS measurement received, 2: Accelerometer received}.
The times since the last received meaurement of the GPS and the accelerometer are defined as 
**dt_gps** and **dt_accel** respectively.
1. Depending on the state of the sensor flag, you should implement the following functionalities 
using both the functions **KF_sensor_fusion** and **KF_state_propagation**:
- When no measurement is received, only propagate the current optimal Kalman filter state by the 
provided input time interval **dt_state_prop** to yield the new Kalman Filter state (**X_est**) and 
prediction covariance estimates (**P_est**).
- When either a GPS or an Accelerometer measurement is received, propagate the current optimal 
Kalman Filter state by the time since the most recent previous measurement (either **dt_accel** or 
**dt_gps**). Then call the correct variables **R**, **H** and **Z** for the measured quantity and 
call **KF_sensor_fusion** to return the new Kalman Filter state (**X_est**) and prediction covariance
(**P_est**) estimates. 
2. Return the estimates **X_est** and **P_est** for each case.

To test your implementation, first set **self.use_ground_truth_measurement = True** and compare your Kalman Filter estimate to the ground truth using the plots generated at the end of the run.
If you are happy with the filtering performance and obtain a result similar to that in the figure below, you can proceed to Part 2.

FIGURE (Filter Peformance compared with Ground truth)

Part 2 - Deployment and Tuning
------------------------------

Now, let us run the PID controller with the activated noisy measurements and a running Kalman Filter. For this part, ensure that **self.use_ground_truth_measurement = False**, **self.use_noisy_measurement = False** and **self.use_accel_only = False**.

The key tuning parameter for the Kalman Filter is the Process Covariance. In our implementation, the process covariance is affected by the coefficient variable **self.q_tr**.
This parameter describes the uncertainty associated with the classical Kalman Filter assumption that the drone undergoes motions with constant acceleration over a single prediction timestep.
In simpler words:
- If **self.q_tr = 0**, we assume that the drone undergoes motions which perfectly match the piecewise constant acclereration assumption. Therefore, the Kalman Filter will rely heavily on our model prediction to provide an accurate state estimate.
- If **self.q_tr >> 0**, we assume that the drone undergoes motions which are different to the piecewise constant accleration assumption. Therefore, with a higher **self.q_tr**, the Kalman Filter will rely more heavily on the noisy sensor measurements to provide a more accurate state estimate.

Starting with **self.q_tr = 0**, increase **self.q_tr** by small increments and investigate how this affects the behavior of the drone in the parcours.

When you feel you have reached satisfactory performance and a low run completion time, you can compare your result to the performance below:

FIGURE (KF run performance)

Total completion time: 

Part 3 - Relying on the Accelerometer
----------------------------------

As a last investigation, let us look at what happens when we only measure accelerations from the 
acclerometer but do not correct our position estimates
with exact GPS measurements.

For this, within your implemented Kalman Filter class, set **self.use_accel_only = True** and 
re-run the simulation.

Your drone movement should show a noticeable change after 10 seconds, similar to this scenario:



Why does this happen?

As we only propagate our GPS measurements but never correct our state estimate with a true (be it noisy) position estimate, the position and velocities are determined primarily from integration of the accelerometer.
The position and velocity estimates therefore "drift" away from the true value as the uncertainty becomes larger and larger over time. This is called sensor drift and is a commonly observed phenomenon when working with accelerometers.

====================================================================================§
Any questions about the exercise, please contact Julius Wanner (julius.wanner@epfl.ch).