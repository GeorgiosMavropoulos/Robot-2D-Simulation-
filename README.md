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


## Project structure
├── main.py # demo runner: runs and animates the gain sets, prints results, runs tests
├── requirements.txt # pygame, matplotlib
├── README.md
├── .gitignore
└── robot_pid/
├── init.py # marks the folder as a package
├── pid.py # PID controller
├── robot.py # unicycle robot model (Forward Euler)
├── map.py # occupancy grid, obstacles, coordinate conversion
├── planner.py # A* path planner
└── simulator.py # closed-loop simulation, metrics, plots, pygame animation

#Requirements
- Python 3.9
- - pygame
- matplotlib

## How to run

1. Create and activate a virtual environment in the project root:
   - Windows (PowerShell): `python -m venv .venv` then `.venv\Scripts\Activate.ps1`
   - macOS / Linux: `python3 -m venv .venv` then `source .venv/bin/activate`
2. Install the dependencies: `pip install -r requirements.txt`
3. Run from the project root: `python main.py`

## Components

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


```python
pid = PID(kp, ki, kd, output_limits=(low, high))
command = pid.update(error, dt)   # one control step
pid.reset()                       # clear integral and previous error
```

- P, I and D terms. The derivative is 0 on the first call.
- Output is clamped to `output_limits`.
- Anti-windup by conditional integration: when the output saturates, the integral change from that step is undone.
- Input validation: `ValueError` for non-finite error, `dt <= 0` or inverted limits; `TypeError` for non-numeric gains.

### Robot (`robot_pid/robot.py`)

```python
robot = Robot(x, y, theta, linear_velocity, angular_velocity)
robot.step(dt)
```

- State: `x`, `y` in metres, `theta` in radians, wrapped to [-pi, pi).
- Forward Euler integration: the position is updated with the current heading, then the heading is updated.
- Inputs are validated: all values must be finite, and `dt` must be positive.
- The model is deterministic, so the same gains always give the same run.

### Map (`robot_pid/map.py`)

- Grid of `rows x cols` cells: `0` is free, `1` is an obstacle. Each cell is `cell_size` metres wide (default 0.2 m).
- Cells are `(row, col)` tuples. World positions are `(x, y)` in metres, with `x` along columns and `y` along rows.
- `build_default_map()` builds the fixed obstacle layout used by all experiments, so every gain set runs on the same path.

### A* planner (`robot_pid/planner.py`)

- 4-connected neighbours, grid heuristic, path reconstruction from `came_from`.
- Returns a list of cells from start to goal, or `None` if no path exists.

### Simulator (`robot_pid/simulator.py`)

- `Simulator(heading_pid, speed_pid, start_cell, goal_cell, dt, threshold, t_max)` builds the map, plans the A* path, and converts it to world waypoints.
- `run_simulation()` runs the control loop and returns a log with `t`, `x`, `y`, `theta`, `heading_error`, `v_com`, `omega_com` and `reached_goal`.
- Control loop, one step per `dt`:
  1. Compute the heading error to the active waypoint.
  2. `omega = heading_pid.update(heading_error, dt)`
  3. `v = speed_pid.update(distance_to_goal, dt) * max(0, cos(heading_error))`. The cosine term slows the robot when it is not facing the target.
  4. Apply `v` and `omega` to the robot and step it forward.
- Metrics: `time_to_goal`, `path_length`, `cross_track_error` (mean and max distance from the planned path), and `metrics()` which returns all of them.
- `animate()` shows the planned path (blue), the actual trail (red) and the robot (green) in pygame.
- `plot_comparison()` saves a matplotlib figure with one subplot per run.
- `export_metrics_to_csv()` writes the metrics of all runs to a CSV file.

## Experiments

All runs use the same map, start cell `(0, 0)`, goal cell `(19, 19)`, `dt = 0.1 s`, `threshold = 0.1 m` and `t_max = 60 s`.

| Run   | Heading PID (kp, ki, kd) | Speed PID (kp, ki, kd) |
|-------|--------------------------|------------------------|
| under | (12.0, 0.01, 0.02)       | (0.4, 0.1, 0.1)        |
| over  | (4.0, 0.01, 1.5)         | (0.03, 0.01, 2.1)      |
| tuned | (7.0, 0.01, 0.4)         | (0.03, 0.01, 2.1)      |

Output limits: heading omega in [-2, 2] rad/s, speed v in [0, 1] m/s.


