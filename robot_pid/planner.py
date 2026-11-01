"""
    AStar algorithm with Manhattan heuristic method
"""
import math
from map import Map
from typing import Any
class AStartPlanner:

    def __init__(self,map):
        """
           Initialized with an object from map class. Runs a validation method to raise an error if map
           is not an object from Map()
           Args:
                map -> Map()

           
        """
        #validate that map is an object from Map class
        self._validate_init_arg(map)
        self.map = map



    @staticmethod
    def _validate_init_arg(map):
        """
           Validates that map is a valid object from map class

           Args: 
                map -> Map

           Raises:
                   TypeError: if map is not an instance of class Map

         
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
        


    def heuristic(self,cell:tuple[int,int],goal:tuple[int,int])->list[tuple[int,int]]:
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
        """Finds the shortest path from start to goal using the A* algorithm.

        Args:
            start: The starting grid coordinate as (x, y).
            goal: The target grid coordinate as (x, y).

        Returns:
            A list of tuples representing the path from start to goal if found,
            or a string message ('Cannot find the path') if no path exists.
        """

         #validate that start/goal is a tuple [int,int], not boolean and a finite number
        self._validate_point(start, "cell")
        self._validate_point(goal, "goal")
        
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        #open list contains the cells the robot hasn't crossed yet
        open_list = [start]
        closed_list = []

        #parent cell
        came_from = {}

        #main loop
        while open_list:
          #find the cell with smaller score
          current = min(open_list, key=lambda cell: f_score[cell])

          if current == goal:
              #if reached the goal, construct the path
              return reconstruct_path(came_from,current)

          #remove the current cell from open list and append it into close list
          closed_list.append(current)
          open_list.remove(current) #remove current from openlist

          #check the neighbor cells if they are free
          for neighbor in self.map.neighbors(current):
              if not self.map._is_free_cell(neighbor) or neighbor in closed_list:
                  #go to next cell if cell is not free
                  continue

              #cost to move from current neighbor to the next one
              tentative_g_score = g_score[current] + 1

              #if we have found a better path for this neighbor. use get to return inf if path not exist
              if tentative_g_score < g_score.get(neighbor, float('inf')):
                  #document this path as the best till now
                  came_from[neighbor] = current
                  g_score[neighbor] = tentative_g_score
                  f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)

                  #if algorithm find for 1st time this neighbor validate if its free
                  if neighbor not in open_list:
                       open_list.append(neighbor)

          #if open_list is empty return that algorithm didn't find the path
          
          return 'Cannot find the path'
         

