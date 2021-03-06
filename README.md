# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

In this project we will implement a Model Predictive Control to drive the car around the track. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

---

#### Video of the car driving around the simulator track available here (https://youtu.be/rXklOh1W6dI) 

- The yellow is a polynomial fitted to waypoints and the green line represents the x and y coordinates of the MPC trajectory.
- For the simulator I used 640 x 480 screen resolution, with a graphic quality of fastest.
- My code is built based on the SDC ND class lessons code and quizzes.

#### I learned how to tune the right weights for the different parts of the cost function, dealing with latency between actuations commands, and better understanding of MPC from the great discussions on the SDC ND community on Slack/Forums and students articles on medium.com. 

## Vehicle Kinematic Model

Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. We want to derive a model that captures how the state evolves over time, and how we can provide an input to change it. 


### The state variables
The simulator gives us the below state variables of the car and a series of waypoints with respect to the arbitrary global map coordinate system which we can use to fit a polynomial used to estimate the curve of the road ahead.
#### px
The current location of the vehicle in the x-axis of an arbitrary global map coordinate system.

#### py
The current location of the vehicle in the y-axis of an arbitrary global map coordinate system.

#### psi
The current orientation of the vehicle

#### v
The current velocity of the vehicle


### The Actuators
Actuator inputs allow us to control the vehicle state. Most cars have three actuators: the steering wheel, the throttle pedal and the brake pedal. For simplicity we'll consider the throttle and brake pedals as a singular actuator.

#### delta
Steering value or turn angle of the vehicle. The angle is between -25 and 25 degrees.

#### a
Throttle/Brake value with negative values signifying braking and positive values signifying acceleration where a can take value between and including -1 and 1.

### Errors
A controller actuates the vehicle to follow the reference trajectory, within a set of design requirements. One important requirement is to minimize the error between the reference trajectory and the vehicle’s actual path. You can minimize this error by predicting the vehicle’s actual path and then adjusting the actuators to minimize the difference between the prediction and the reference trajectory. That's how to use the kinematic model to predict the vehicle’s future state.

#### Cross Track Error cte
The difference between our desired position and actual position. 

#### Orientation Error epsi
The difference between our desired orientation and actual orientation. Can be computed using an arctan to the derivative of the fitted polynomial function at point px = 0 to get the angle to which we should be heading.


#### The model we’ve developed:
px(t+1) = px(t) + v(t) * cos(psi(t)) * dt

py(t+1) = py(t) + v(t) * sin(psi(t)) * dt

psi(t+1) = psi(t) + v(t) / Lf * (-delta) * dt

v(t+1) = v(t) + a(t) * dt

cte(t+1) = cte(t) - v(t) * sin(epsi(t)) * dt

epsi(t+1) = epsi(t) +  v(t) / Lf * (-delta) * dt

where Lf measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle , the slower the turn rate.

cte = f(px) - py

epsi = psi - atan(f_derv(px))

where f is the fitted polynomial function, f_derv is the derivative of f.


###  N & dt values
The prediction horizon is the duration over which future predictions are made and it is the product of two variables, N and dt. N is the number of timesteps in the horizon. dt is how much time elapses between actuations. These are hyperparameters we need to tune for each model predictive controller we build.

Too small N will not enable us to look ahead enough to plan well our path especially curves, and too big N will cause us to look ahead too much than we actually need with the cost of calculating too much before the feedback. I choose N=10 after many trails and errors.

For dt, I choose dt = 0.1 sec as it's the latency between actuation commands which helps with the way I deal with the latency in implementing my MPC.


### Cost function
The objective is to minimize the cross track error cte and the orientation error epsi, so our cost should be a function of how far these errors are from 0. 

We will also penalize the vehicle for not maintaining the reference velocity to avoid it from halting in the middle of the reference trajectory.

We also include to the cost the control inputs, this will help us to penalize the magnitude of the control input as well as the change-rate of the control inputs. The main purpose of that is to avoid erratic driving by jerking the steering wheel or consecutive hard acceleration/braking.

The different weights for the cost function are tuned by trial-and-error and with the help of discussions over the SDCND Slack/forums community.


### Polynomial Fitting and MPC Preprocessing
The simulator gives us the state variables of the car and a series of waypoints with respect to the arbitrary global map coordinate system which we need to transform to the vehicle coordinates system..I used a 3rd order polynomial as an estimate of the current road curve ahead. 

The errors calculated: For the current cte error equals the fitted polynomial function at px = 0. For the current orientation error epsi equals the arctan(f_derv) where f_derv is the derivative of the fitted polynomial.


### Model Predictive Control with Latency
Model Predictive Control (MPC) uses an optimizer to find the control inputs that minimize the cost function.

The MPC algorithm:

Setup:

    1- Define the length of the trajectory, N, and duration of each timestep, dt.
    2- Define vehicle dynamics and actuator limitations along with other constraints.
    3- Define the cost function.

Loop:

    1- We pass the current state as the initial state to the model predictive controller.
    2- We call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver we'll use is called Ipopt.
    3- We apply the first control input to the vehicle.
    4- Back to 1.


#### To deal with the 0.1 sec latency:-  

I choose the dt = 0.1 sec and instead of using the given state, we first compute the state with the delay using our kinematic model before feeding it to the solving object.

In the code:
```
          ////////////////////////////////////////////////////////////////////////////
          /////*** Getting the state considering the latency****//////////////////////
      
          
          const double current_px = 0.0 + v * dt;
          const double current_py = 0.0; // Car is going straight ahead the x-axis
          const double current_psi = 0.0 + v * (-delta) / Lf * dt;
          const double current_v = v + a * dt;
          const double current_cte = cte + v * sin(epsi) * dt;
          const double current_epsi = epsi + v * (-delta) / Lf * dt;
          
          Eigen::VectorXd current_state(6);
          current_state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;
          
          //////////////////////////////////////////////////////////////////////////////
          ////*** Getting next predicted states and actuators using MPC***//////////////
          mpc.Solve(current_state, K);
          steer_value = mpc.steering;
          throttle_value = mpc.throttle;
          

```

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
