# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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

## Model

The car is represented by the Bicycle Motion Model, which simplifies the state and actuation equations without sacrificing the accuracy of the model.

Using 2D trigonometry, one can easily get the following state update equations where x and y denote the position of the vehicle in the global coordinate system, psi is the heading of the vehicle, v is the velocity along the current heading, delta_t is the steering angle, Lf is the distance between the center of mass of the vehicle and its front steering wheel, and a is its acceleration.

[updateEquations]: ./UpdateEquations.PNG "Update Equations"
![updateEquations][updateEquations]

The actuation is controlled by two parameters, delta, which represents the steering angle from the steering wheel, and a, which represents the acceleration resulting from the throttle and brake pedals.

The trajectory is fitted by a third degree polynomial, which can model most of the roads on which the car would run.

## Parameter Tuning
There are several parameters that contribute to a stable and effective MPC controller.
Two of these are related to the length and frequency of the prediction:
1. N - number of steps to predict
2. dt - the interval between each prediction and actuation step
From these values, we get T = N x dt, which is the number of seconds in the future until which the model calculates and predicts the state. a large dt value may make the calculations inaccurate, while a small dt might not be long enough to finish all the calculations required for prediction and actuation at each step. A large T means we try to predict a long time into the future, which becomes more inaccurate with increasing values of T.
For a reference speed of 100 MPH = 44.7 m/s, values of N = 10 and dt = 0.1 were found to strike a good balance between the tradeoffs described above. This produced a T = 1 s, which maps the waypoints to a reasonable 45 meters ahead of the vehicle. 
In fact with larger T values (e.g. N = 20, dt = 0.1), the added waypoints sometimes unnecessarily affect the actuation commands and turn out to be worse than if only closer waypoints were taken into account. When T is too small, the car sometimes does not react to sharp turns quickly enough and can go off track at higher speeds.

Other parameters that affect the MPC are the gains of the various errors and actuation parameters in the cost function.
1. Cross-Track Error - how far is the vehicle from the center of the lane
2. Psi Error - how far off is the vehicle's heading from the ideal value
3. Velocity Error - how far away is the vehicle's velocity to the desired value
4- Steering Magnitude - how big is the steering value to be sent. Large values are penalized to avoid unstable back-and-forth steering 
5- Acceleration Magnitude - how big is the throttle value to be sent. Large values are penalized to avoid constant changes in speed
6- Steering Change - how much is the current steering value different from the last one sent. Large values are penalized to avoid sudden changes in steering values to have smooth turns.
7- Acceleration Change - how much is the current throttle value different from the last one sent. Large values are penalized to avoid sudden changes acceleration to have smooth speed changes.

## Latency Handling
In order to simulate the real-world latency between getting sensor data, calculating state info, and actuation commands taking effect, an artificial delay was added in the simulator.
Even though the MPC worked while without the delay, once the delay was introduced, the controller started sending larger and larger steering values because the correction would not be made on time. This cannot be solved by simply tuning the parameters described above.
In order to handle the latency, the initial state values at each steps are changed. Using the current state values and the same state prediction equations, the predicted state after the latency can be approximated, and the controller can use those values to send more accurate actuation commands.
