# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Overview
In this project, I implemented a PID controller in C++ to maneuver the vehicle around the track.
The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Simulator
You can download the simualtor [here](https://github.com/udacity/self-driving-car-sim/releases). Then, choose "Project 4: PID Controller" ![Project 4: PID Controller](/images/PID_window.PNG)
## Reflection
### The effect of the P, I, D component of the PID algorithm
- The proportional component depends only on the difference between the set point and the process variable. This difference is referred to as the Error term. The proportional gain (Kc) determines the ratio of output response to the error signal. In general, increasing the proportional gain will increase the speed of the control system response. However, if the proportional gain is too large, the process variable will begin to oscillate. If Kc is increased further, the oscillations will become larger and the system will become unstable and may even oscillate out of control.
- The integral component sums the error term over time. The result is that even a small error term will cause the integral component to increase slowly. The integral response will continually increase over time unless the error is zero, so the effect is to drive the Steady-State error to zero. Steady-State error is the final difference between the process variable and set point. A phenomenon called integral windup results when integral action saturates a controller without the controller driving the error signal toward zero.
- The derivative component causes the output to decrease if the process variable is increasing rapidly. The derivative response is proportional to the rate of change of the process variable. Increasing the derivative parameter will cause the control system to react more strongly to changes in the error term and will increase the speed of the overall control system response. Most practical control systems use very small derivative parameter, because the Derivative Response is highly sensitive to noise in the process variable signal. If the sensor feedback signal is noisy or if the control loop rate is too slow, the derivative response can make the control system unstable

Source: [http://www.ni.com/white-paper/3782/en/](http://www.ni.com/white-paper/3782/en/)

### Video of the car
[Video sample](https://youtu.be/3C0sR5jeMy4)  
This video has been made with a previous version of my code. There are some minor changes in respect to my final code (```max_steer``` and ```throttle``` in the ```main.cpp``` file).

### Final hyperparameters
I decided to manually tune the hyperparameters. I first tried to implement twiddle, but it was difficult to implement and didn't give me a good result. Watching what other students did, I decided to discard the twiddle implementation and I focus my effort on optimizing manually the hyperparameters. 
I choose to set P to 0.35, I to 0.00005 and D to 18. 
I first tried with all the three hyperparameters close to 1, but the car went off road. I had to drastically diminish I to make the car stay on track. To optimize driving I had to drastically increase D to make the car stay on track recovering from errors.
I, then, played a little with ```throttle``` and ```steer_value``` to increase the speed of the car and increasing its performance. You can read the code in the ```main.cpp``` file (lines 70:88).
The car reached the speed of 60 mph.
