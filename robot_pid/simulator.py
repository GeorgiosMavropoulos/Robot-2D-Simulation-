"""
Simulator module.

Runs a closed-loop path tracking simulation of a unicycle robot.
A* plans a grid path, which is converted to world waypoints. Two PID
controllers then drive the robot along it: one for heading (omega) and
one for speed (v). The module also provides metrics, plots and a
pygame animation.
"""
from __future__ import annotations
import matplotlib.pyplot as plt 

from planner import AStarPlanner

from map import Map
import pygame
import csv
import math
from robot import Robot, angle_wrap

class Simulator:
    """
    Closed-loop path tracking simulator for a unicycle robot.

    On creation, the simulator builds the occupancy map, plans a path with
    A* from start_cell to goal_cell, and converts the path to world waypoints
    in metres.

    The control loop runs in steps of dt seconds. The heading PID turns the
    robot toward the active waypoint. The speed PID sets the forward speed
    from the remaining distance to the goal, scaled by cos(heading_error)
    so the robot slows down when it is not facing the target.

    Results are returned as a log dictionary. Metrics (time, path length,
    cross-track error) are computed from that log.
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
             #validate initialization arguments
             self._validate_init_args(heading_pid, speed_pid, start_cell, goal_cell, dt, threshold, t_max)
             self.map = Map.build_default_map()
             self.planner = AStarPlanner(self.map)
             self.waypoints = self.generate_waypoints(start_cell, goal_cell)
             self.heading_pid = heading_pid
             self.speed_pid = speed_pid
             self.dt = dt
             self.threshold = threshold
             self.t_max = t_max


    def _validate_init_args(self, heading_pid, speed_pid, start_cell, goal_cell, dt, threshold, t_max):
          """
          Validates types, interfaces, structural boundaries, and mathematical constraints.

          This helper ensures the input PID controllers possess the correct interface 
          update and reset methods, checks that cellular grid destinations are formatted 
          as valid index integer coordinate pairs, and verifies that time slices, limits, 
          and tolerance ranges are strictly positive finite numbers.

          Args:
              heading_pid: The controller object passed to govern angular steering loops.
              speed_pid: The controller object passed to govern linear movement velocities.
              start_cell (tuple): Matrix index row/column tracking location for spawning.
              goal_cell (tuple): Matrix index row/column tracking location for terminal target.
              dt (float): Continuous simulation step discrete increment interval.
              threshold (float): Euclidean error arrival boundary margin scale.
              t_max (float): Maximum simulation sequence time restriction ceiling.

          Raises:
              TypeError: If controller objects lack callable interfaces, cells aren't 
                          integer tuples, or parameters are not standard numeric values.
              ValueError: If cell coordinates are negative, or if parameters (dt, threshold, t_max)
                          are zero, negative, or non-finite.
          """
          #interface structural verification checking required operational method hooks
          if not hasattr(heading_pid, "update") or not hasattr(heading_pid, "reset"):
              raise TypeError("heading_pid object must implement callable update() and reset() methods.")
          if not hasattr(speed_pid, "update") or not hasattr(speed_pid, "reset"):
              raise TypeError("speed_pid object must implement callable update() and reset() methods.")

          #cell coordinates layout type tracking validation checks
          for cell_name, cell in [("start_cell", start_cell), ("goal_cell", goal_cell)]:
              if not isinstance(cell, (tuple, list)) or len(cell) != 2:
                  raise TypeError(f"{cell_name} must be a sequence container of exactly 2 elements.")
              if isinstance(cell[0], bool) or isinstance(cell[1], bool):
                  raise TypeError(f"{cell_name} indices cannot be formatted as boolean types.")
              if not isinstance(cell[0], int) or not isinstance(cell[1], int):
                  raise TypeError(f"{cell_name} tracking grid matrix indices must be strict integers.")
              if cell[0] < 0 or cell[1] < 0:
                  raise ValueError(f"{cell_name} layout coordinate elements cannot be negative numbers.")

          #numeric profile constraints mapping validations loop
          numeric_params = [("dt", dt), ("threshold", threshold), ("t_max", t_max)]
          for param_name, value in numeric_params:
              if isinstance(value, bool):
                  raise TypeError(f"Parameter '{param_name}' cannot be passed as a boolean flag.")
              if not isinstance(value, (int, float)):
                  raise TypeError(f"Parameter '{param_name}' must be formatted as an integer or float numeric type.")
              if not math.isfinite(value):
                  raise ValueError(f"Parameter '{param_name}' must be passed as a valid finite real number.")
              if value <= 0:
                  raise ValueError(f"Parameter '{param_name}' must evaluate to a strictly positive non-zero metric.")

              

    
    @staticmethod
    def calculate_heading(current_pose, target_waypoint, tolerance=1e-3) -> float | None:
        """
        Calculates the heading (theta) that points from the current position to a target.

        Note: `tolerance` is a numerical guard for "the robot is exactly on the target"
        and is independent of the simulator's `threshold` (waypoint arrival radius).

        Args:
            current_pose (tuple): Current position as (x, y).
            target_waypoint (tuple): Target position as (x, y).
            tolerance (float): If the distance to the target is below this value, the
                heading is undefined. Defaults to 1e-3.

        Returns:
            float | None: Heading in radians in [-pi, pi], or None if the robot is
                within `tolerance` of the target (the caller must keep its current heading).

        Raises:
            TypeError: If the inputs are not sequences of two numbers.
            ValueError: If any coordinate is NaN or infinite.
        """
        try:
            x_curr, y_curr = current_pose
            x_goal, y_goal = target_waypoint
        except (TypeError, ValueError) as e:
            raise TypeError(
                f"Inputs must be sequences of two numbers (x, y). Error: {e}"
            ) from e

        if not all(math.isfinite(v) for v in (x_curr, y_curr, x_goal, y_goal)):
            raise ValueError("Coordinates must be finite real numbers (not NaN or Inf).")

        delta_x = x_goal - x_curr
        delta_y = y_goal - y_curr

        if math.hypot(delta_x, delta_y) < tolerance:
            return None

        return math.atan2(delta_y, delta_x)


   
    def generate_waypoints(self, start_cell: tuple[int, int],goal_cell: tuple[int, int]) -> list[tuple[float, float]]:
                       
        """
        Plans a grid path from start to goal and converts it to world coordinates.

        Args:
            start_cell (tuple): Start position as (row, col).
            goal_cell (tuple): Goal position as (row, col).

        Returns:
            list[tuple[float, float]]: Waypoints in metres as [(x1, y1), (x2, y2), ...].
                Contains at least one point (the start) even when start equals goal.

        Raises:
            ValueError: If the planner returns no path, or returns something that is not a list.
        """
        cells = self.planner.a_star(start_cell, goal_cell)

        if cells is None:
            raise ValueError(f"No path found from {start_cell} to {goal_cell}.")
        if not isinstance(cells, list):
            raise ValueError(f"Planner returned {type(cells).__name__}, expected a list of cells.")

        waypoints = []
        for row, col in cells:
            x, y = self.map.cell_to_world(row, col)
            waypoints.append((x, y))

        return waypoints

    def run_simulation(self) -> dict:
        """
        Runs the closed-loop path tracking simulation along the waypoints.

        The loop advances until the final waypoint is reached or t_max is exceeded.
        PID controllers are reset at the start so repeated runs give the same result.

        Returns:
            dict: Time series 't', 'x', 'y', 'theta', 'heading_error', 'v_com', 'omega_com',
                and the flag 'reached_goal' (bool).
        """
        self.heading_pid.reset()
        self.speed_pid.reset()

        x0, y0 = self.waypoints[0]

        #already at the goal (start equals goal): nothing to track.
        if len(self.waypoints) < 2:
            return {"t": [0.0], "x": [x0], "y": [y0], "theta": [0.0],
                    "heading_error": [0.0], "v_com": [0.0], "omega_com": [0.0],
                    "reached_goal": True}

        theta0 = self.calculate_heading((x0, y0), self.waypoints[1])
        if theta0 is None:
            theta0 = 0.0  #first two waypoints coincide: keep a default heading

        robot = Robot(x0, y0, theta0, 0, 0)

        data = {"t": [0.0], "x": [robot.x], "y": [robot.y], "theta": [robot.theta],
                "heading_error": [0.0], "v_com": [0.0], "omega_com": [0.0]}

        index, last, t, reached_goal = 1, len(self.waypoints) - 1, 0.0, False

        while t < self.t_max:
            target = self.waypoints[index]
            if math.dist((robot.x, robot.y), target) <= self.threshold:
                if index == last:
                    reached_goal = True
                    break
                index += 1
                continue

            desired = self.calculate_heading((robot.x, robot.y), target)
            if desired is None:
                #robot is numerically on the target: keep the current heading.
                desired = robot.theta

            error = angle_wrap(desired - robot.theta)
            omega = self.heading_pid.update(error, self.dt)

            dist_goal = math.dist((robot.x, robot.y), self.waypoints[last])
            v = self.speed_pid.update(dist_goal, self.dt)
            v *= max(0.0, math.cos(error))

            robot.linear_velocity = v
            robot.angular_velocity = omega
            robot.step(self.dt)

            t += self.dt
            data["t"].append(t)
            data["x"].append(robot.x)
            data["y"].append(robot.y)
            data["theta"].append(robot.theta)
            data["heading_error"].append(error)
            data["v_com"].append(v)
            data["omega_com"].append(omega)

        data["reached_goal"] = reached_goal
        return data



    @staticmethod
    def distance_point_to_segment(p, a, b)->float:
        """
        Calculates the shortest Euclidean distance from a point to a line segment.

        This function determines the minimum distance between a query point P and a 
        finite straight-line segment bounded strictly by endpoints A and B. It handles 
        the edge case where the segment is zero-length (A and B collapse into the same point)
        and clamps the projection parameter to restrict calculations within the physical segment bounds.

        Args:
            p (tuple): The query point coordinate as (x, y).
            a (tuple): The start point coordinate of the line segment as (x, y).
            b (tuple): The end point coordinate of the line segment as (x, y).

        Returns:
            float: The minimum Euclidean distance from point P to the segment AB.

        Raises:
            TypeError: If any of the inputs are not sequences of two numbers (x, y).
            ValueError: If any coordinate value is invalid (e.g., NaN or Infinity).
    """

        #type validation and structural error handling
        try:
            px, py = p
            ax, ay = a
            bx, by = b
        except (TypeError, ValueError) as e:
            raise TypeError(
                f"Inputs must be sequences of two numbers (x, y). Error: {e}"
            ) from e

        #value validation (check for NaN or infinity)
        if not all(math.isfinite(val) for val in [px, py, ax, ay, bx, by]):
            raise ValueError("Coordinates must be finite real numbers (not NaN or Inf).")

        #compute the directional vector coordinates of the segment from A to B (Vector AB)
        abx, aby = b[0] - a[0], b[1] - a[1]

        #calculate the squared magnitude/length of the segment AB (||AB||^2)
        length_sq = abx**2 + aby**2

         #edge case: if A and B are identical, the segment collapses into a single localized point
        #the shortest distance is simply the direct Euclidean distance from P to point A
        if length_sq == 0:
            return math.dist(p, a)

        #compute the vector coordinates stretching from segment start A to point P (Vector AP)
        apx, apy = p[0] - a[0], p[1] - a[1]

        #calculate projection factor 't' using the Vector Dot Product equation: (AP . AB) / ||AB||^2
         #this evaluates where the perpendicular projection of P drops along the infinite line AB
        t = (apx * abx + apy * aby) / length_sq

        t = max(0, min(1, t))

        #reconstruct the absolute world frame coordinates of the closest point on segment AB
        closest_point = (a[0] + t * abx, a[1] + t * aby)

        return math.dist(p, closest_point)



    def time_to_goal(self, data:dict)->int|str:
        """Extracts the total elapsed travel time if the target goal was reached.

        This method checks the trajectory state profile results from the simulation.
        If the robot successfully completed its route, it returns the final recorded
        timestamp. If the robot ran out of time or drifted off course, it returns a
        failure status message string.

        Args:
            data (dict): The trajectory logging dictionary returned by run_simulation.
                         Must contain the key 'reached_goal' (bool) and 't' (list of floats).

        Returns:
            float | str: The final elapsed time in seconds if reached_goal is True,
                         or the descriptive failure string if False.

        Raises:
            TypeError: If the data argument is not a dictionary payload.
            KeyError: If required tracking keys 'reached_goal' or 't' are missing.
            ValueError: If the time tracking sequence array 't' is empty.
        """
        #type validation to ensure input payload is a structured dictionary
        if not isinstance(data, dict):
            raise TypeError("Input data must be a dictionary payload result.")

        #key validation to verify necessary logging elements are present
        if "reached_goal" not in data or "t" not in data:
            raise KeyError("Dictionary must contain both 'reached_goal' and 't' tracking keys.")

        #value validation to check that the timeseries list is not completely empty
        if not isinstance(data["t"], list) or len(data["t"]) == 0:
            raise ValueError("Time tracking array 't' cannot be empty or structurally invalid.")

        if data["reached_goal"]:
            return data['t'][-1]
        else:
            return 'Robot failed to reach the goal'

    def path_length(self, data:dict)->float:
        """
        Computes the total Euclidean path length traversed by the robot.

        This method integrates the sequential point-to-point distances along the 
        recorded trajectory history to determine the total physical odometer distance 
        traveled by the mobile robot during the simulation run.

        Args:
            data (dict): The trajectory logging dictionary returned by run_simulation.
                         Must contain tracking keys 'x' (list of floats) and 'y' (list of floats).

        Returns:
            float: The total path distance traveled in world units.

        Raises:
            TypeError: If the data argument is not a dictionary payload.
            KeyError: If required coordinate tracking keys 'x' or 'y' are missing.
            ValueError: If coordinate arrays are empty, structurally mismatched, or invalid.
        """
        #type validation to ensure input payload is a structured dictionary
        if not isinstance(data, dict):
            raise TypeError("Input data must be a dictionary payload result.")

        #key validation to verify necessary localized coordinate tracks are present
        if "x" not in data or "y" not in data:
            raise KeyError("Dictionary must contain both 'x' and 'y' tracking keys.")

        #extract underlying trajectory data structures
        x_coords = data["x"]
        y_coords = data["y"]

        #type validation ensuring target profiles are stored within standard sequence lists
        if not isinstance(x_coords, list) or not isinstance(y_coords, list):
            raise TypeError("Coordinate tracks 'x' and 'y' must be formatted as lists.")

        #value validation confirming that both parallel arrays match in overall structural length
        if len(x_coords) != len(y_coords):
            raise ValueError("Coordinate arrays 'x' and 'y' must be equal in length.")

        #initialize absolute cumulative trajectory distance counter
        total_distance = 0.0

        #edge case safety check: return zero if there are fewer than two recorded coordinates
        if len(x_coords) < 2:
            return total_distance

        #incrementally accumulate step-by-step distances across successive historical path markers
        for i in range(len(x_coords) - 1):
            #calculate displacement step change differences over the localized time slice interval
            dx = x_coords[i + 1] - x_coords[i]
            dy = y_coords[i + 1] - y_coords[i]
            
            #compute standard Euclidean vector magnitude step size using math.hypot
            step_distance = math.hypot(dx, dy)
            
            #add calculated slice change directly into cumulative odometer storage registry
            total_distance += step_distance

        #return definitive structural trajectory profile length
        return total_distance


    def cross_track_error(self, data:dict)->tuple[float, float]:
        """Computes the mean and maximum Cross-Track Error (CTE) of the robot.

        This method calculates the perpendicular path tracking deviation of the 
        robot at every recorded simulation time step relative to the nearest 
        geometric waypoint segment. It then computes aggregated statistics to 
        evaluate overall steering precision.

        Args:
            data (dict): The trajectory logging dictionary returned by run_simulation.
                         Must contain tracking keys 'x' (list of floats) and 'y' (list of floats).

        Returns:
            tuple[float, float]: A pair containing (mean_error, max_error) across 
                                 the entire historical trajectory profile.

        Raises:
            TypeError: If the data argument is not a dictionary payload.
            KeyError: If required coordinate tracking keys 'x' or 'y' are missing.
            ValueError: If coordinate arrays are empty, mismatched, or if no 
                        waypoint segments are generated.
        """
        #type validation to ensure input payload is a structured dictionary
        if not isinstance(data, dict):
            raise TypeError("Input data must be a dictionary payload result.")

        #key validation to verify necessary localized coordinate tracks are present
        if "x" not in data or "y" not in data:
            raise KeyError("Dictionary must contain both 'x' and 'y' tracking keys.")

        #extract underlying tracking coordinates
        x_coords = data["x"]
        y_coords = data["y"]

        #type validation ensuring target profiles are stored within standard sequence lists
        if not isinstance(x_coords, list) or not isinstance(y_coords, list):
            raise TypeError("Coordinate tracks 'x' and 'y' must be formatted as lists.")

        #value validation confirming that both parallel positional arrays match in length
        if len(x_coords) != len(y_coords):
            raise ValueError("Coordinate arrays 'x' and 'y' must be equal in length.")

        #value validation ensuring there are actual position entries to evaluate
        if len(x_coords) == 0:
            raise ValueError("Coordinate tracking lists cannot be empty.")

        #value validation ensuring there are enough waypoints to formulate line segments
        if len(self.waypoints) < 2:
            raise ValueError("At least two waypoints are required to construct track segments.")

        #pair adjacent trajectory waypoints into localized linear segment boundaries
        segments = list(zip(self.waypoints[:-1], self.waypoints[1:]))
        errors = []
        
        #iterate through every concurrent spatial coordinate recorded during the run
        for x, y in zip(x_coords, y_coords):
            robot_pos = (x, y)
            
            #find the absolute minimum tracking distance from (x, y) across ALL segments
            min_segment_dist = min(
                self.distance_point_to_segment(robot_pos, seg[0], seg[1]) for seg in segments
            )
            errors.append(min_segment_dist)
            
        # compute and return aggregated metrics as a descriptive (mean_error, max_error) tuple
        mean_error = sum(errors) / len(errors)
        max_error = max(errors)
        
        return mean_error, max_error

          
    def metrics(self, data):
      mean_cte, max_cte = self.cross_track_error(data)
      return {"time": self.time_to_goal(data),
              "path_length": self.path_length(data),
              "cte_mean": mean_cte,
              "cte_max": max_cte} 

    def world_to_screen(self, x:float, y:float, scale:float)->tuple[float, float]:
        """
        Transforms continuous world frame coordinates into discrete 2D screen pixels.

        This method scales real-world coordinates to fit onto a display window. 
        It also handles an essential graphics inversion edge case: in world coordinates, 
        the Y-axis points upward, whereas standard computer graphics screens position 
        the origin (0,0) at the top-left corner, meaning screen coordinates point downward.

        Args:
            x (float): The horizontal position component in the world coordinate frame.
            y (float): The vertical position component in the world coordinate frame.
            scale (float): Scaling factor multiplier (e.g., pixels per world unit).

        Returns:
            tuple[float, float]: Position coordinate mapped to screen space as (px, py).

        Raises:
            TypeError: If x, y, or scale are not real numbers.
            ValueError: If coordinate values or scale factors are infinite or NaN,
                        or if the scale multiplier is zero or negative.
        """
        #type and value validation to guarantee inputs are safe finite real numbers
        if not all(isinstance(val, (int, float)) for val in [x, y, scale]):
            raise TypeError("Coordinates (x, y) and scale factor must be numeric integers or floats.")
            
        if not all(math.isfinite(val) for val in [x, y, scale]):
            raise ValueError("Coordinates and scale must be finite real numbers (not NaN or Inf).")

        if scale <= 0:
            raise ValueError("The display scale parameter must be a positive non-zero value.")

        #calculate total dynamic map height boundary in continuous world unit tracking space
        height = self.map.rows * self.map.cell_size

        #scale the horizontal vector directly to map standard pixel widths
        px = x * scale
        
        #invert the vertical height axis so that the visual matrix renders right side up
        py = (height - y) * scale

        #return the computed pixel tuple coordinates to the drawing canvas pipeline
        return (px, py)


    def draw_map(self, screen, scale:float):
        """Renders the grid map onto the Pygame display screen surface.

        This method iterates through the rows and columns of the underlying grid map, 
        constructs a corresponding pixel rectangle for each cell scaled to display sizes,
        maps the cell state (free vs. obstacle) to a specific visual color, and 
        draws the grid elements onto the display screen canvas pipeline.

        Args:
            screen (pygame.Surface): The active Pygame screen display window surface to draw on.
            scale (float): Scaling factor multiplier converting world dimensions into screen pixels.

        Raises:
            TypeError: If the scale parameter is not a numeric value.
            ValueError: If the scale multiplier is zero or negative.
        """
        #type and value validation to ensure the scale parameter is safe
        if not isinstance(scale, (int, float)):
            raise TypeError("The display scale parameter must be a numeric integer or float.")
        if scale <= 0:
            raise ValueError("The display scale parameter must be a positive non-zero value.")

        #define RGB color values for rendering states
        free_color = (255, 255, 255) #white color for accessible pathways
        obstacle_color = (40, 40, 40) #dark gray/black color for impassable wall barriers

        #calculate discrete screen cell dimensions in terms of pixels
        cell_px = self.map.cell_size * scale
        
        #traverse the entire matrix configuration space row by row
        for row in range(self.map.rows):
            for col in range(self.map.cols):
                #construct the target bounding rect element for Pygame layout rendering
                rect = pygame.Rect(
                    round(col * cell_px), 
                    round(row * cell_px),
                    math.ceil(cell_px), 
                    math.ceil(cell_px)
                )
                
                #check cell value configuration array indices to assign correct render color
                if self.map.grid_map[row][col] == 1:
                    color = obstacle_color
                else:
                    color = free_color
                    
                #draw the scaled rectangular matrix unit grid onto the canvas layout surface
                pygame.draw.rect(screen, color, rect)


    def animate(self, data:dict, scale=150, fps=None):
          """Animates the historical path tracking trajectory playback using Pygame.

          This method visually updates step-by-step frames representing the robot moving 
          along the path. It renders the underlying map layout matrix, displays the 
          global planned target path in blue, maps a running history line of actual 
          travel coordinates in red, and draws a robot directional tracker head.

          Args:
              data (dict): The trajectory logging dictionary returned by run_simulation.
                          Must contain 't', 'x', 'y', and 'theta' coordinate tracks.
              scale (float): Scaling factor multiplier to scale world coordinates to pixels. 
                            Defaults to 150.
              fps (float | None): Playback framing rate control ceiling. If None, it automatically 
                                  locks and maps sync speed values matching the simulation 1/dt rate.

          Raises:
              TypeError: If the data input argument structure is not a dictionary payload.
              KeyError: If mandatory timeseries metrics lists are missing from the data layout.
          """
          #type validation checking dataset parameters
          if not isinstance(data, dict):
              raise TypeError("Animation tracking playback dataset must be an encapsulated dict dictionary.")

          #key validation checking required positional arrays
          required_keys = ["t", "x", "y", "theta"]
          if not all(key in data for key in required_keys):
              raise KeyError("Input dictionary is missing mandatory keys: 't', 'x', 'y', or 'theta'.")

          #initialize frame pacing limits from system delta changes if no manual limits are requested
          if fps is None:
              fps = 1.0 / self.dt

          #initialize the graphical media system environment modules
          pygame.init()
          try:
              #calculate exact window viewport layout resolution scales
              width = int(self.map.cols * self.map.cell_size * scale)
              height = int(self.map.rows * self.map.cell_size * scale)
              screen = pygame.display.set_mode((width, height))
              pygame.display.set_caption("Robot path tracking")
              clock = pygame.time.Clock()

              #pre-render static structural map grids onto an isolated background surface buffer
              map_surface = pygame.Surface((width, height))
              self.draw_map(map_surface, scale)

              #transform planned waypoint sequences cleanly from world positions to display pixel arrays
              planned = [self.world_to_screen(x, y, scale) for x, y in self.waypoints]
              i, last, running = 0, len(data["t"]) - 1, True

              #execute main visual playback event loops
              while running:
                  #flush device operating background interface signals to capture terminate calls
                  for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        running = False

                  #refresh window canvas by blasting pre-cached background surface layers on top
                  screen.blit(map_surface, (0, 0))
                  
                  #draw the global structural optimal path layout track in solid blue
                  pygame.draw.lines(screen, (0, 0, 255), False, planned, 3)

                  #process current structural history coordinate paths up to loop frame tracker 'i'
                  actual_trail_world = list(zip(data["x"][:i+1], data["y"][:i+1]))
                  
                  #convert history tracking elements straight to pixel coordinates
                  actual_trail_pixels = [self.world_to_screen(wx, wy, scale) for wx, wy in actual_trail_world]

                  #render the trailing actual robot footprint path tracking layout history line in red
                  if len(actual_trail_pixels) >= 2:
                      pygame.draw.lines(screen, (255, 0, 0), False, actual_trail_pixels, 2)

                  #extract localized real-time tracking metrics at index position 'i'
                  rx = data["x"][i]
                  ry = data["y"][i]
                  rtheta = data["theta"][i]

                  #map robot center space origin directly inside frame pixels
                  robot_center_px = self.world_to_screen(rx, ry, scale)

                  #define structural radius parameters for drawing operations
                  robot_radius_px = max(5, int(0.15 * scale)) #keep the robot size proportional to scale
                  heading_line_length_m = 0.25 #length of the orientation pointer line in meters

                  #calculate the absolute outer edge boundary coordinates of the nose vector in meters
                  line_end_x_world = rx + heading_line_length_m * math.cos(rtheta)
                  line_end_y_world = ry + heading_line_length_m * math.sin(rtheta)

                  #convert the heading vector end-point directly to pixel layouts
                  robot_nose_px = self.world_to_screen(line_end_x_world, line_end_y_world, scale)

                  #draw the robot chassis footprint frame body container circle as a distinct bright green
                  pygame.draw.circle(screen, (0, 255, 0), (int(robot_center_px[0]), int(robot_center_px[1])), robot_radius_px)

                  #draw a dark orientation layout pointer line directly from robot center to nose location
                  pygame.draw.line(screen, (0, 100, 0), robot_center_px, robot_nose_px, 3)

                  #flip buffers to render full vector layouts onto display screens
                  pygame.display.flip()
                  
                  #lock thread timing execution cycles to stay inside framing speed rates
                  clock.tick(fps)
                  
                  #advance history array counters to load subsequent data profiles
                  if i < last:
                      i += 1
          finally:
              #safely shut down engine pipeline hooks during termination sequences
              pygame.quit()


    @staticmethod
    def plot_comparison(runs, filename="simulation_comparison.png"):
        """Plots trajectory comparisons side-by-side and saves the figure as an image.

        This method maps out a discrete spatial grid tracking visualization across multiple 
        simulation execution runs. It draws background occupancy obstacle layers, overlays 
        planned baseline waypoints in red alongside actual position metrics curves in blue, 
        and extracts localized controller parameter state gain settings directly into sub-chart labels.

        Args:
            runs (list[tuple]): A sequence collection containing run parameters formatted as 
                                tuples of (label_str, simulator_obj, data_dict).
            filename (str): The destination local storage file track path where the output figure 
                            will be rendered and written. Defaults to "simulation_comparison.png".

        Raises:
            TypeError: If runs is not formatted as an iterable list structure.
            ValueError: If the runs collection payload is completely empty.
        """
        #type and value validation checking matching comparison configurations
        if not isinstance(runs, list):
            raise TypeError("The comparison parameter runs must be provided as a structured list.")
        if len(runs) == 0:
            raise ValueError("The sequence collection 'runs' cannot be empty.")

        #initialize responsive grid subplot dimensions based on active elements tracking structures
        fig, axes = plt.subplots(1, len(runs), figsize=(5 * len(runs), 5), squeeze=False)

        #iterate concurrently across spatial plot axis channels and structured datasets
        for ax, (label, sim, data) in zip(axes[0], runs):
            W = sim.map.cols * sim.map.cell_size
            H = sim.map.rows * sim.map.cell_size

            #render underlying occupancy grids (extent maps top grid row 0 cleanly to coordinates max height)
            ax.imshow(sim.map.grid_map, cmap="Greys", vmin=0, vmax=1, extent=[0, W, 0, H])

            #extract individual coordinate vectors for drawing optimal pathways
            wx = [pt[0] for pt in sim.waypoints]
            wy = [pt[1] for pt in sim.waypoints]

            #overlay optimal reference point markers on top of grid lines as bright red dotted marks
            ax.plot(wx, wy, 'ro-', markersize=4, label='Planned Waypoints')

            #overlay dynamic historical vehicle trajectory pathways as a solid blue contour curve
            ax.plot(data["x"], data["y"], 'b-', linewidth=2, label='Actual Trajectory')

            #safely extract gain attributes from heading and speed controller entities
            h_pid = sim.heading_pid
            s_pid = sim.speed_pid
            
            #compile parameters into a detailed structural multiline title string
            title_text = (
                f"{label}\n"
                f"Heading PID (Kp:{h_pid.kp:.2f}, Ki:{h_pid.ki:.2f}, Kd:{h_pid.kd:.2f})\n"
                f"Speed PID (Kp:{s_pid.kp:.2f}, Ki:{s_pid.ki:.2f}, Kd:{s_pid.kd:.2f})"
            )
            
            #apply text properties and enforce geometric coordinate scale uniformity properties
            ax.set_title(title_text, fontsize=9)
            ax.set_xlabel("X Coordinate (m)")
            ax.set_ylabel("Y Coordinate (m)")
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", alpha=0.5)

        #tighten layout distributions to remove wasteful white padding gaps
        fig.tight_layout()
        
        #persist visualization figure metrics layout onto the disk drive asset registries
        fig.savefig(filename, dpi=300)
        print(f"Comparison plot figure successfully exported and saved as: '{filename}'")
        
        #render graphics scene layout windows to screen displays
        plt.show()


    @staticmethod
    def export_metrics_to_csv(runs, filename="simulation_metrics.csv"):
        """
        Aggregates analytical metrics from all simulation runs and exports them to a CSV file.

        This method extracts performance indices including target resolution status, absolute tracking runtime, 
        cumulative odometer path length, and mean/maximum cross-track deviations across all run profiles 
        and structures them inside a clean CSV tabular database file.

        Args:
            runs (list[tuple]): A sequence collection containing run parameters formatted as 
                                tuples of (label_str, simulator_obj, data_dict).
            filename (str): The destination local storage path where the CSV output file will be written. 
                            Defaults to "simulation_metrics.csv".

        Raises:
            TypeError: If runs is not an iterable list payload.
            ValueError: If no simulation configurations are provided inside the tracking inputs array.
        """
        

        #type and value validation checking payload characteristics
        if not isinstance(runs, list):
            raise TypeError("The tracking argument runs must be formatted as an accessible sequence list.")
        if len(runs) == 0:
            raise ValueError("The tracking simulation list cannot be empty.")

        #open target output stream context handlers with clean newlines tracking parameters
        with open(filename, mode='w', newline='', encoding='utf-8') as csv_file:
            #define database schema headers covering core kinematics and control stability tracking elements
            fieldnames = [
                "Run_Label", 
                "Reached_Goal", 
                "Total_Time_Seconds", 
                "Path_Length_Meters", 
                "Mean_CTE_Meters", 
                "Max_CTE_Meters"
            ]
            
            #instantiate standard writing interfaces
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            
            #write structural column headers onto line index zero
            writer.writeheader()

            #process each simulation dataset sequence incrementally
            for label, sim, data in runs:
                #calculate total elapsed duration metrics profiles safely
                time_metric = sim.time_to_goal(data)
                
                #calculate cumulative spatial displacement distances traversed
                distance_metric = sim.path_length(data)
                
                #calculate tracking alignment error margins profiles
                mean_cte, max_cte = sim.cross_track_error(data)

                #populate table records matching target columns keys
                writer.writerow({
                    "Run_Label": label,
                    "Reached_Goal": data["reached_goal"],
                    "Total_Time_Seconds": time_metric,
                    "Path_Length_Meters": round(distance_metric, 4),
                    "Mean_CTE_Meters": round(mean_cte, 4),
                    "Max_CTE_Meters": round(max_cte, 4)
                })

        print(f"Performance metrics tabular datasheet successfully written to: '{filename}'")



