"""
Simulator module
"""
from .pid import PID
from .planner import AStarPlanner
from .robot import Robot
from .map import Map
import pygame
import math
from .robot import Robot, angle_wrap

class Simulator:
    """
    Manages and executes the kinematic simulation of a mobile robot.
    
    This class utilizes an A* path planner to generate structural grid waypoints,
    converts them to world coordinates, and executes a closed-loop control system
    using two independent PID controllers: one for heading (angular velocity) 
    and one for speed (linear velocity).
    """
    def __init__(self, heading_pid, speed_pid, start_cell=(0, 0), goal_cell=(19, 19),
             dt=0.1, threshold=0.1, t_max=60.0):
             """
             Initializes the simulator with the required controllers and parameters.

        Args:
            heading_pid: The PID controller object governing steering/angular velocity.

            speed_pid: The PID controller object governing forward/linear velocity.

            start_cell (tuple): The starting grid position mapped as (row, col). Defaults to (0, 0).

            goal_cell (tuple): The target destination grid position mapped as (row, col). Defaults to (19, 19).

            dt (float): The discrete simulation time step in seconds. Defaults to 0.1.

            threshold (float): Distance tolerance radius to verify waypoint arrival. Defaults to 0.1.

            t_max (float): Maximum allowed runtime ceiling for the simulation in seconds. Defaults to 60.0.
        """
             self.map = Map.build_default_map()
             self.planner = AStarPlanner(self.map)
             self.waypoints = self.generate_waypoints(start_cell, goal_cell)
             self.heading_pid = heading_pid
             self.speed_pid = speed_pid
             self.dt = dt
             self.threshold = threshold
             self.t_max = t_max



    @staticmethod
    def calculate_heading(current_pose, target_waypoint, tolerance=1e-3)->float:
      """
      Calculates the target theta (heading) for the robot to face a waypoint.

      Args:
          current_pose (tuple): The robot's current position as (x, y).
          target_waypoint (tuple): The goal position as (x, y).
          tolerance (float): Distance threshold below which the robot is 
              considered to have arrived at the waypoint. Defaults to 1e-3.

      Returns:
          float: The target orientation (theta) in radians between -pi and pi.
              Returns None if the robot is already at the waypoint.

      Raises:
          TypeError: If inputs are not iterable or missing required coordinates.
          ValueError: If coordinate values are invalid (e.g., NaN or Inf).
      """
      #type validation and structural error handling
      try:
          x_curr, y_curr = current_pose
          x_goal, y_goal = target_waypoint
      except (TypeError, ValueError) as e:
          raise TypeError(
              f"Inputs must be sequences of two numbers (x, y). Error: {e}"
          ) from e

      #value validation (check for NaN or infinity)
      if not all(math.isfinite(val) for val in [x_curr, y_curr, x_goal, y_goal]):
          raise ValueError("Coordinates must be finite real numbers (not NaN or Inf).")

      #calculate distance differences
      delta_x = x_goal - x_curr
      delta_y = y_goal - y_curr

      #Robotics Edge Case: Robot is already at the target point
      # Calculating Euclidean distance to check against tolerance
      distance = math.hypot(delta_x, delta_y)
      if distance < tolerance:
          #returning None signals to your controller to maintain current heading
          return None

      #compute safe theta
      return math.atan2(delta_y, delta_x)


   
    def generate_waypoints(self,start_cell:tuple[int,int],goal_cell:tuple[int,int])->list[tuple[float, float]]:
        """
        Generates a list of world coordinates (waypoints) from start to goal.

        Args:
            start_cell (tuple): The starting grid position as (row, col).
            goal_cell (tuple): The target grid position as (row, col).

        Returns:
            list[tuple[float, float]]: A list of coordinates representing 
                the path in the world frame, e.g., [(x1, y1), (x2, y2), ...].

        Raises:
            ValueError: If the planner fails to find a valid path or 
                does not return a list of cells.
        """

        cells = self.planner.a_star(start_cell,goal_cell)

        #return an error message if input cells are not a list
        if not isinstance(cells,list):
            raise ValueError('Invalid value for cells')

        
        waypoints =[]

        #unpack cells and convert to world coordinates
        for cell in cells:
            row,col = cell
            x,y = self.map.cell_to_world(row,col)
            waypoints.append((x,y))           

        return waypoints

    def run_simulation(self):
      """
      Executes the closed-loop tracking simulation along the generated waypoint trajectory.

        The simulation loop advances incrementally until either the final waypoint 
        is successfully reached or the time elapsed crosses the t_max safety threshold.

        Returns:
            dict: An architectural log dictionary recording timeseries arrays for state profiles 
                  (t, x, y, theta), errors (heading_error), commands (v_com, omega_com), 
                  and a boolean flag signifying target goal resolution status.
        """

      #reset heading pid and pid on each run in order not to accumulate previous values
      self.heading_pid.reset()
      self.speed_pid.reset()

      #extract the initial starting position coordinates from the waypoints array
      x0, y0 = self.waypoints[0]

      #calculate the target theta
      theta0 = self.calculate_heading((x0, y0), self.waypoints[1])
      #instanciate the localized Robot object tracking (x, y, theta, linear_vel, angular_vel)
      robot = Robot(x0, y0, theta0, 0, 0)

      #create data logging dictionary structure with initial t=0 step entries
      data = {"t": [0.0], "x": [robot.x], "y": [robot.y], "theta": [robot.theta],
              "heading_error": [0.0], "v_com": [0.0], "omega_com": [0.0]}

      #instanciate  tracking index pointer, total destination length index, clock, and status flag
      index, last, t, reached_goal = 1, len(self.waypoints) - 1, 0.0, False

      #check if the robot has arrived within the threshold bounds of the current target waypoint
      while t < self.t_max:
          #if current index matches the terminal waypoint, target tracking is successfully completed
          target = self.waypoints[index]
          if math.dist((robot.x, robot.y), target) <= self.threshold:
              if index == last:
                  reached_goal = True
                  break
              #increment look-ahead pointer index to lock on to the next subsequent waypoint segment
              index += 1
              continue
          # calculate the absolute desired heading vector pointing directly from the robot to the active waypoint
          desired = self.calculate_heading((robot.x, robot.y), target)

          #compute rotational steering error wrapped tightly into the localized [-pi, pi] constraint framework
          error = angle_wrap(desired - robot.theta)
          #command the heading PID controller to update and generate the necessary angular velocity (omega)
          omega = self.heading_pid.update(error, self.dt)

          #compute the residual Euclidean distance map directly separating the robot from its terminal goal
          dist_goal = math.dist((robot.x,robot.y),self.waypoints[last])

      
          #query the speed PID controller to calculate base linear velocity based on remaining distance-to-goal
          v = self.speed_pid.update(dist_goal,self.dt)
          #scale linear velocity using cosine of heading error. This slows down the robot 
          # when it's facing away from the target to prevent it from drifting off course during sharp turns.
          v *= max(0, math.cos(error))
          #apply computed commands directly to the robot's physical model attributes
          robot.linear_velocity = v
          robot.angular_velocity = omega
          #propagate the robot state dynamics forward in time by one discrete time-step (dt)
          robot.step(self.dt)

          #increment  simulation execution runtime clock tracker
          t += self.dt
          #append newly updated parameters into log tracking indices
          data["t"].append(t)
          data["x"].append(robot.x)
          data["y"].append(robot.y)
          data["theta"].append(robot.theta)
          data["heading_error"].append(error)
          data["v_com"].append(v)
          data["omega_com"].append(omega)

      #map terminal condition state success flag status within returned dataset payload
      data["reached_goal"] = reached_goal
      return data
    

