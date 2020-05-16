# Self-Driving Car Engineer Nanodegree Program
<p align="center">
<img src="output/output.gif" width="400" height="270" border="10">
</p>

## PID Controller

Given a driving simulator which provides the cross track error (CTE) and the velocity (mph), the goal of this project is to develop a PID controller in c++ that successfully drives the vehicle around the track. In order to navigate the car around the track. I have implemented PID controller for steering angle control.

## Tuning

The most important part of the project is to tune the hyper-parameters. This can be done by different methods such as manual tuning, Zieglor-Nichols tuning, SGD,or Twiddle. I have initially done manual tuning, followed by the twiddle method. Manual tuning is hard but, the process of tuning help us better understand every single effect of PID parameters. The following table summarizes the effect of each parameter on the system.

Parameter | Rise Time    | Overshoot | Settling Time | Steady-state error
--------- | ------------ | --------- | ------------- | ------------------
Kp        | Decrease     | Increase  | Small change  | Decrease
Ki        | Decrease     | Increase  | Increase      | Decrease
Kd        | Small change | Decrease  | Decrease      | No change

### Proportional (P):

This parameter controls the error proportionally. Increasing the proportional gain has the effect of proportionally increasing the control signal for the same level of error. Setting only P control is aggressive and has oscillations.

### Integral (I):

This parameter controls the accumulating error. Addition of this term reduces the steady state error. If there is a bias in the system, the integrator builds and builds, thereby increasing the control signal and driving the error down.

### Derivative (D):

This parameter controls the rate of change of error. Addition of this term reduces the oscillatory effect in the system. With derivative control, the control signal can become large if the error begins sloping upward, even while the magnitude of the error is still relatively small. This anticipation tends to add damping to the system, thereby decreasing overshoot.

The following approach is a best to tune manually:

- Set all gains to zero.
- Increase the P gain until the response to a disturbance is steady oscillation.
- Increase the D gain until the the oscillations go away (i.e. it's critically damped).
- Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
- Set P and D to the last stable values.
- Increase the I gain until it brings you to the setpoint with the number of oscillations desired.

I started initially with (0.1,0.03,2.0) for the steering control. The final parameters for my controllers are:

Parameter | Values   | 
--------- | ------------ | 
Kp        | 0.13     | 
Ki        | 0.0001     | 
Kd        | 1.0 | 


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


--------------------------------------------------------------------------------
