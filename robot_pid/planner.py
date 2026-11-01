from __future__ import annotations
"""A* path planning on a 2D grid map.

This module provides  a path planner that finds the shortest
path between two cells of a grid using the A* search algorithm with the
Manhattan distance as its heuristic. Moves are 4-directional and every move
has a cost of 1.

The planner relies on a Map's object (imported from the map module) that
must expose a neighbors(cell) method returning the free cells adjacent to
cell.
"""
import heapq
from .map import Map
from typing import Any


class AStarPlanner:

    """
        Shortest-path planner for a grid map using A* with Manhattan heuristic.
    
        The planner is created with a Map instance and can then be queried
        any number of times for the shortest path between two grid coordinates.
    
        Attributes:
            map: The Map instance the planner searches on.
    
        Raises:
            TypeError: On construction, if map is not an instance of Map.
        """

    def __init__(self,map):
        """
        Initializes the planner with a map.

        Runs a validation method to raise an error if map is not an object
        from Map().

        Args:
            map: A Map instance.

        Raises:
            TypeError: If map is not an instance of class Map.
        """
        
        #validate that map is an object from Map class
        self._validate_init_arg(map)
        self.map = map



    @staticmethod
    def _validate_init_arg(map):
        """
         
         Validates that map is a valid object from the Map class.

        Args:
            map: The object to validate.

        Raises:
            TypeError: If map is not an instance of class Map.
        """
        
        if not isinstance(map, Map):
            raise TypeError('map must be an instance of class Map')

        
    
    @staticmethod
    def _validate_point(point: Any, name: str) -> None:
        """Validates that a coordinate point is a tuple of two finite integers.

        Args:
            point: The coordinate to validate (expected to be a tuple).
            name: The name of the variable (e.g., 'cell', 'goal') for error messages.

        Raises:
            TypeError: If point is not a 2-tuple, contains booleans, or non-integers.
            
        """

        
        #check if it's a tuple with exactly 2 elements
        if not isinstance(point, tuple) or len(point) != 2:
            raise TypeError(f"{name} must be a tuple with 2 elements")

        
        #check for booleans (since bool is a subclass of int in Python)
        if isinstance(point[0], bool) or isinstance(point[1], bool):
          raise TypeError(f"{name} cannot contain booleans")
        
          
        #check if elements are integers
        if not isinstance(point[0], int) or not isinstance(point[1], int):
            raise TypeError(f"{name} must contain integers")
        


    def heuristic(self,cell:tuple[int,int],goal:tuple[int,int])->int:
        """
            Manhattan Algorithm. Takes start position and computes the shortest length till reach the goal.

            Args:
                 cell -> tuple[int,int]
                 goal -> tuple[int,int]

            Returns:
                    The total horizontal and vertical steps between the two points.
            
            Raises: 
                   TypeError: if cell/goal contain booleans,
                              if cell/goal is not a tuple with 2 integers
                   ValueError: if cell/goal is not a finite number
                             
        
        """

        #validate that cell/goal is a tuple [int,int], not boolean and a finite number
        self._validate_point(cell, "cell")
        self._validate_point(goal, "goal")

        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])



    def a_star(self,start:tuple[int,int],goal:tuple[int,int])->list[tuple[int, int]] | str:
        """
        Finds the shortest path from start to goal using the A* algorithm.

        Args:
            start: The starting grid coordinate as (x, y).
            goal: The target grid coordinate as (x, y).

        Returns:
            A list of tuples representing the path from start to goal if found,
            or a string message ('Cannot find the path') if no path exists.
        """

         #validate that start/goal is a tuple [int,int], not boolean and a finite number
        self._validate_point(start, "start")
        self._validate_point(goal, "goal")
        
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        #open list contains the cells the robot hasn't crossed yet
        open_list = []
        heapq.heappush(open_list, (0, start)) #push the start point position into the list. The 0 is the f-score (the cost)
        came_from ={}

        #main loop
        while open_list:
          #find the cell with smaller score. '_' means that we are not intereste right now for the first item
          _,current = heapq.heappop(open_list) #return the smallest item from the open last.

          if current == goal:
              #if reached the goal, construct the path
              return self.reconstruct_path(came_from,current)

          #check the neighbor cells if they are free
          for neighbor in self.map.neighbors(current):
             
              #for each neighbor blocks of current block add + 1 into the cost, 
              # because we are moving towards one block from current block. the new cost is the current total steps + 1
              tentative_g_score = g_score[current] + 1
              #if current cost is less than the previous cost (or if neighbor has not been visited in the past),
              #  it means this path is shorter to reach the next block.
              if tentative_g_score < g_score.get(neighbor, float('inf')):
                  came_from[neighbor] = current
                  g_score[neighbor] = tentative_g_score

                  #caclulate the total distance from current cell
                  f = tentative_g_score + self.heuristic(neighbor, goal)
                  f_score[neighbor] = f
                 
                  #append to the open list the cost and the neighbor cell
                  heapq.heappush(open_list, (f, neighbor))

        #if open_list is empty return that algorithm didn't find the path
          
        return 'Cannot find the path'


    @staticmethod
    def reconstruct_path(came_from: dict[tuple[int, int], tuple[int, int]],current:tuple[int,int])-> tuple[int,int]:
        """Reconstruct the path when find the shortest path. Reversing the path

        since the algorithm searches backwards.

        Args: 
             came_from: Dictionary mapping coordinates to their parents.
             current: The goal coordinate to start tracing backwards from.

        Returns:
             The full path as a list of tuples from start to goal.
        """
        #validate the current cell        
        AStarPlanner._validate_point(current,"current cell")

        #validate the parent cell
        if isinstance(came_from,bool) or not isinstance(came_from,dict):
            raise TypeError('came_from must be a valid dictionary.')

        path = [current]

        while current is not None:
            parent_cell = came_from.get(current)

            if parent_cell is None:
                break
            current = parent_cell
            path.append(current)

        path.reverse() #reverse the path since a start went backwords
        return path

        
