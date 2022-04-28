# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Project Instructions

In the previous project you built a path planner for the autonomous vehicle. Now you will build the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

In the directory [/pid_controller](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller)  you will find the files [pid.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid.cpp)  and [pid.h](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid.h). This is where you will code your pid controller.
The function pid is called in [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp).

### Step 1: Build the PID controller object
Complete the TODO in the [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid_controller.h) and [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid_controller.cpp).

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.
### Step 2: PID controller for throttle:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp), complete the TODO (step 2) to compute the error for the throttle pid. The error is the speed difference between the actual speed and the desired speed.

Useful variables:
- The last point of **v_points** vector contains the velocity computed by the path planner.
- **velocity** contains the actual velocity.
- The output of the controller should be inside [-1, 1].

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 3: PID controller for steer:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp), complete the TODO (step 3) to compute the error for the steer pid. The error is the angle difference between the actual steer and the desired steer to reach the planned position.

Useful variables:
- The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
- **yaw** gives the actual rotational angle of the car.
- The output of the controller should be inside [-1.2, 1.2].
- If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 4: Evaluate the PID efficiency
The values of the error and the pid command are saved in thottle_data.txt and steer_data.txt.
Plot the saved values using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You might need to install a few additional python modules: 

```
pip3 install pandas
pip3 install matplotlib
```

![carla](https://user-images.githubusercontent.com/22205974/165790472-5d6ac888-76f9-4f26-89ad-c14ad6253a47.PNG)

PID (proportional integral derivative) controllers use a control loop feedback mechanism to control process variables. Manually tuning parameters by observing the simulated responses, the optimal values were found by trying first the P, then the I and then the D values one by one. In the simulation, the reactions of the vehicle were monitored.

## What is the effect of the PID according to the plots, how each part of the PID affects the control command? 

The figure below shows the steering control errors. The oscillations in the timeline, show the vehicle is trying to learn its environment, while trying to avoid obstacles. Part of the errors may be due to latency or other incompatibilities between the controller and simulation. The agent has to acquire new experiences in the environment during which it can update its policy and/or value function. The policy specifies for a state, or to change an action value associated with a state, it has to move to that state, act from it, possibly many times, and experience the consequences of its actions

![carla1](https://user-images.githubusercontent.com/22205974/165791224-0dc94ff0-c9c6-4a8f-99b5-37cb23c2e253.PNG)

The figure below

![carla2](https://user-images.githubusercontent.com/22205974/165791663-4bfab971-a95a-41ed-94be-9d809fc6745e.PNG)

## Elaborate a critical analysis of the controller.

The questions are answered with justified explanations:

## How would you design a way to automatically tune the PID parameters? 

Manual tuning starts with setting the integral and derivative values to 0. The proportional is increased until the controller starts to oscillate. Testing and adjusting a PID controller for stability, to achieve adequate reaction times. Autotuning can eliminate the trial and error of the manual tuning approach. Automatic tuning of PID parameters through tuning a control loop or with the Twiddle algorithm. Twiddle is an algorithm that tries to find a good choice of parameters p for an algorithm A that returns an error. When twiddle variable set to true, simulator runs the car with confidents till the maximum steps set initially continue through the twiddle algorithm. After competition of each iteration, simulator reset to initial stage from the start until maximum steps have been achieved.

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller? Find at least 2 pros and cons for model free versus model based.

## Pros
      The advantage of PID controller is its feasibility and easy to be implemented.

      PID gain can be designed just based on the system tracking error.

      Best controller to use when the processes can’t be modeled.

## Cons
      Sensitive to parameter variations, can be unstable

      The controller generally has to balance

      PID controller has low robust ability when the system encounters to multiple challenges.
