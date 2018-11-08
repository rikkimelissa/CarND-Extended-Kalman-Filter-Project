# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project I utilized an extended kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project required obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Input values provided by the simular included observed measurements from lidar and radar.
Out values provided an estimated position in x and y and root mean squared error for position and velocity in both directions.

Required steps for implementing this extended Kalman filter included:
- Initializing the Kalman filter position vector with the first sensor measurmeents
- Modifying F and Q matrices prior to the prediction step based on the elapsed time between measurements
- Calling the update step for lidar or radar measurements
- Linearizing update equations for radar measurements by calculating a Jacobian matrix




---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `




