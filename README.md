# Robot-2D-Simulation
Robot 2D simulation with A* algorithm, and PID Control

# Robotics  University of Sunderland.

# Delivery:
1. 2D robot simulator in Python, using pygame
2. Implementation of path planner (A* search).
3. PID controller class to control heading (angular velocity), forward velocity (speed).
4. Integration of PID with the planner so that the robot can follow a planned path in the simulator.
5. Experiments with at least three different PID gain sets (under-damped, over-damped, well-tuned).
6. Recorded metrics: path length, cross track error, time to reach goal


PID Class:
Provides a robust PID (Proportional-Integral-Derivative) controller
equipped with conditional integration (clamping) to prevent integral windup 
during actuator saturation. It includes strict parameter validation at 
initialization and runtime.
Initialize: PID(kp,ki,kd,output_limits)


Robot Class:
Provides the Robot class designed to simulate a moving robot on a
two-dimensional plane (x, y, theta). Using the Forward Euler Integration method,
the robot's state updates over discrete time steps based on linear and angular
velocities. The module features robust type and value verification blocks to
mitigate non-finite inputs or unstable simulation clocks.
Initialize: Robot(x,y,theta,linear_velocity,angular_velocity)


