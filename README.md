# Control MPC Project
In this project I implemented a MPC controller to maneuver an autonomous driving car around a track in a simulator.
The code is written in C++, because this language provides great performance in terms of memory and speed.
This work is part of the Self-Driving Car Engineer Nanodegree Program at Udacity.

---

In this project, I implemented in C++ a MPC controller to maneuver an autonomous driving car around a track in a simulator. 


MPC Controller Project - Self-Driving Car Engineer Nanodegree - Udacity



[//]: # (Image References)
[simulation]: ./img/turn80mph.png

## Results

### Video

[A video of the results can be found in YouTube in this link.](https://youtu.be/qdTNysVirsI)

### Simulator
In order to test the controller I used a visualization tool provided by Udacity that simulates the motion of a car around a track. The C++ program receives by telemetry the state of the vehicle in real time (position, velocity, orientation) and the waypoints. Then it passes to the simulator the actuator values (steering angle and throttle).

![Simulator][simulation]


### Implementation

The program receives in each step a list of waypoints of the desired trajectory (yellow line). Then, it calculates the reference path (green line) that the car should follow to align with the waypoints path (yellow line). Considering the kinematic model of the car, it solves the non-linear problem to output the actuators values for each time step and pass them to the simulator input. In this case, the actuators consist of the steering angle and the throttle. After a lot of tuning, the vehicle is able to drive safely at a maximum of 93mph, speeding down to 60mph when it makes a turn. It is also possible to adjust a little bit the parameters in order to increase the speed even more.

In order to go further with this project, I also implemented an algorithm to tune the reference velocity dynamically, so that the car speeds down when a turn is coming. This is a natural reaction of a human driver that is not naturally considered by the MPC algorithm.

Furthermore, there is also a delay time of 100ms between the command of the actuators and the actual execution in the simulator, which emulates what happens in the real world. This was taken into account in the algorithm achieving a great result.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
