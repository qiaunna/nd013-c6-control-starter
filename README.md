# nd013-c6-control-starter
# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Solution
A PID controller is implemented and integrated into the provided framework for throttle and steering control. Several fixes are added to the framework and simulation client to make the control smoother and more stable. PID controller (proportional–integral–derivative controller) parameters are handled separately for steering and throttle. Errors are also calculated separately. I tuned the parameters for the PID by observing the responses.

Due to insufficient hardware for simulation, an application was developed on the remote desktop provided by UDACITY. The results are given below. </br>
[pid_test](https://www.youtube.com/watch?v=PPTg27vqDOs&ab_channel=MelikeTanr%C4%B1kulu "pid")

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



### Solution

PID (proportional integral derivative) controllers use a control loop feedback mechanism to control process variables. While optimizing the PID parameter, the optimal values were found by trying first the P, then the I and then the D values one by one. In the simulation, the reactions of the vehicle were monitored.

Due to insufficient hardware for simulation, an application was developed on the remote desktop provided by UDACITY. The results are given below. </br>
[pid_test](https://www.youtube.com/watch?v=PPTg27vqDOs&ab_channel=MelikeTanr%C4%B1kulu "pid")

### Evaluation

A PID controller is implemented and integrated into the provided framework for throttle and steering control. Several fixes are added to the framework and simulation client to make the control smoother and more stable. PID controller (proportional–integral–derivative controller) parameters are handled separately for steering and throttle. Errors are also calculated separately. I tuned the parameters for the PID by observing the responses. When calculating the error, the yaw angle of the vehicle and the closest x and y points to the vehicle in the simulation and where the vehicle is at x and y were taken as reference.

The following figure shows the values in case of the simulation, the reference speed is calculated by the behavior planner. The x axis shows iterations. It can be seen that it takes a long time to reach the desired speed (the controller is damped) and even despite this, some oscillation is clearly visible.  It can be seen that initially the error is reduced quickly (caused by the proportional term), but then the equalibrium is reached slowly - this is caused by the slow buildup of the integral term.



The Figure below shows the steering control error and its output in a scene with obstacles. There are three parts in the timeline where the error is huge, these are the 3 moments when the car runs through obstacles and has to change lanes. Part of the error here may be due to latency or other incompatibilities between the controller and simulation - for example, the root cause may be that the controller is planning too far ahead (takes the last waypoint).



### Questions 

* PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?

Being independent of the model of PID reduces complexity. On the other hand, it needs to be rearranged for each new vehicle. We can't specify constraints because we don't have a model. mpc can be given as an example of model dependent algorithms.

* How would you design a way to automatically tune the PID parameters?

Parameter optimization is performed using the twiddle algorithm, which is based on making iterative changes to each parameter while an improvement is seen, reducing the magnitude of the change if better values are not found.
