# Model Predictive Control Project
Udacity Self-Driving Car Engineer Nanodegree Program: Term 2
[Master project repo](https://github.com/udacity/CarND-MPC-Project)

This project implements a model predictive control (MPC) algorithm to drive a (simulated) autonomous vehicle arond a track. The MPC algorithm is fed a set of waypoints that trace the center of the road ahead, and returns actuator values for the steering and acceleration/braking. The core of the algorithm is a physics-based model of the vehicle's motion and a cost function derived from the cross-track error and other components.

This project uses the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Project structure

The project consists of the following primary components:
* src/main.cpp - Responsible for interfacing with the simulator and reading (simulated) waypoints for the lane center
* src/MPC.cpp, .h - Responsible for implementing the model predictive control algorithm and holding all values associated with it

## MPC concepts

The MPC control paradigm re-envisions the control problem as an optimization problem about the vehicle's future trajectory. A physics-based model determines how the vehicle will respond to actuator inputs (steering and throttle). The algorithm is given a set of waypoints (or equivalently, a polynomial describing the lane center ahead of the vehicle), and finds the optimum set of actuator values at each timestep to minimize a cost function relative to those waypoints. The first set of actuator values are then fed to the vehicle, and the remainder are discarded.

At the next timestep, the state of the vehicle is measured again, and the waypoint list (equivalently, the polynomial fit) is updated reflecting the vehicle's new position. The controller solves the optimization problem again, returning only the first set of actuator values to the vehicle. This loop repeats continuously.

The reason the future actuator values identified by the optmizer are discarded rather than used reflects the fact that the uncertainty in the vehicle's state grows with time - the model isn't perfect - and it is more accurate to re-measure the state after each time step and re-run the optimization problem.

## Waypoints and polynomial fitting

In my implementation, the Udacity simulator passes a message with six waypoints ahead of the vehicle, describing the center of the track. The waypoints are in global coordinates, and the first task is to map them to the vehicle's frame of reference, in which the vehicle is centered at the origin, and has an orientation angle of zero (lines 113-121 of main.cpp). 

These mapped waypoints are then passed to a polynomial fit function, which returns the coefficients of a third-order polynomial describing them. Using the polynomial coefficients, I can calculate the CTE and orientation error. Because of the coordinate transformation, these equations reduce to a simple form (lines 133-134 of main.cpp).

## Handling latency

Real vehicles suffer from latency of actuators, meaning that the actual steering and throttle may not respond instantly to the commands from the controller. MPC can handle this situation well, by using the physics-based model to predict where the vehicle will be after the latency interval, and calculating the CTE and orientation error. I implement this using the model equations, which are in a simplified form because of the coordinate transformation (lines 141-148 of main.cpp). Following this, I package the post-latency state variables and errors and pass them to the MPC solver.

## Solver setup and constraints

The MPC solver sets up a vector to hold the state variables, errors, and actuator values for all future timesteps within the simulation. Choosing the number of timesteps, and the time interval between them, is a balance between increasing the precision of the analysis and minimizing computational time (which must be kept below the timestep interval to operate in real-time). These are set by the variables N and dt in MPC.cpp; after some experimentation I settled on using 10 timesteps of 0.1s each.

The solver initializes all values in the vector to zero except for the state and error values for the first (current) timestep. It then sets the upper and lower bounds these values can take; for all non-actuator values these are the maximum system number, while the steering is limited to +/- 25 degrees (normalized by the vehicle effective length) and the throttle is limited to +/-1. 

Next, the solver sets the upper and lower constraints on all values. These are simply the values themselves for the current timestep, and zero for all future timesteps. I set the constraint equations later, to reflect the model equations of motion.

Finally, I call the solve function with all constraints specified, and the optimizer returns the problem solution with the future actuator values and model-predicted vehicle state at each timestep.

## Constraint equations from model

The optimizer imposes constraints on the state and error values at all future timesteps based on the physics model of the vehicle motion (lines 105-110 of MPC.cpp). These reflect the physical and geometric effects of the steering angle and throttle values explored by the optimizer. Using these, the optimizer can calculate a cost function and seek to minimize it over the actuator state space.

## Cost function

The optimizer seeks to minimize a cost fucntion, and tuning this function is an important part of getting good performance out of the controller. This involves both determining what variables should be considered, and what weights to give them. Per the classroom discussion, the main factors to include are:

* The cross-track error (CTE), orientation error, and velocity error (vs a setpoint velocity)
* The actuator values (steering and throttle)
* The different in actuator values from the previous timestep

The square of these values are all added to the cost function with individual weights. Using equal weights (of 1.0) produces erratic driving for even relatively low speeds (40 mph), mostly because the solver seems to fail around sharp corners (i.e. it appears to run out of time). 

Following the suggestions in the quiz, increasing the cost of the CTE and orientation error helps stabilize the vehicle. Using weights of 1000 for each, the vehicle is able to navigate the track at 50-60 mph without trouble (the solver fails briefly around corners but recovers well). The vehicle brakes suddenly in some corners, but increasing the weights of the cost function for actuator changes to 100 reduces this.

One important observation was the need to keep the weight of the velocity error small; increasing this too much led to oscillations in which the vehicle would overshoot the reference velocity and then brake suddenly. With a small weight, the velocity tends to stabilize a little below the reference velocity, so that there is a continuous, positive acceleration and minimal or no braking.

After further experimentation, I settled on CTE and orientation weights of 2000, velocity weight of 1, actuator weights of 10, and actuator difference weights of 100. The performance of this is reasonably good, with the exception that the optimizer runs out of time at certain points in the S-curve. The vehicle recovers well from these situations, however, so it drives the lap successfully.

## Speed experiments

I tried running the vehicle with a target velocity of 75 mph; with the weights specified above it got up to 65 mph on the straightaways but struggled with the S curve (with the optimizer running out of time at several timesteps). Reducing the number of timesteps to 8 resulted in erratic behavior (model prediction that appears far off from the waypoint fit) and increasing to 12 resulted in singificantly more optimizer failures. Increasing the weights of the CTE and orientation error seems to have the best effect, although there are still occasional optimizer failures even at weights of 4000. 

Finally, to reduce the compute time and hopefully reduce the number of optimizer time-outs, I defined some helper variables in the constraints calculation (such as velocity times time) to eliminate redundant calculations. This doesn't seem to have had much impact.

---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
