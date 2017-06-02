# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Setup

### Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
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
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


### Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Implementation

### Model

The 3 factors that were considered in this model were the state, actuators, and the model update equations . 

### 1. State

The vehicle's state had 5 parameters:
(1) x: the vehicle's x position
(2) y: the vehicle's y position
(3) psi: the vehicle's orientation
(4) cte: the cross track error
(5) epsi: the vehicle's orientation error

### 2. Actuators

Actuators are inputs that allow us to control the vehicle's state. Most cars have three actuators: the steering wheel, the throttle pedal, and the brake pedal. For this projet, we consider the throttle and brake pedals to be a single actuator (negative values signify braking, while positive values signify acceleration). For this project our actuators are delta (steering angle) and a (acceleration).

### 3. Update Equations

We used the update equations below:
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```


## Timestep Length and Elapsed Duration (N & dt)

Timestep Length (N) is the number of variables optimized by the MPC and is the major driver of computational cost, while the elapsed duration (dt) is how much time is elapsed between actuations. For example, if N were 20 and dt were 0.5, then T would be 10 seconds.

I experimented with different dt values (0.1+) but when I dropped down to 0.05, the vehicle was driving significantly more stable. In our quiz, we had set our N to 25. I quickly that with such a high time step, the solver would in fact be optimizing more variables than necessary and would not be able to execute in the allocated cpu time. 

My final choices for my variables were 12 for my N and 0.05 for my dt.

## Polynomial Fitting and MPC Preprocessing

Waypoints were converted from map coordinates to vehicle coordinates to simplify processing for the MPC. We fit a third degree polynomial with the waypoints.

## Model Predictive Control with Latency

Latency can be thought of as the delay as the actuation command propagates through the system in a car. This latency may be on the order of 100 milliseconds and presents a difficult challenge for some controllers. With MPC, however, we were able to incorporate the latency into the system by predicting the state after 100 ms and then using this new state for our solver. By incorporating the latency, we were able to guarantee that the simulator was receiving the most real0time state of the vehicle. 

### Result
[Here] (https://youtu.be/_0kk6iRglls) is a video showing the car driving around the simulated track. The vehicle is being controled by the modeled MPC
