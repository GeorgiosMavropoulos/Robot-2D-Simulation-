"""
    AStar algorithm with Manhattan heuristic method
"""
import math
from typing import Any
class AStartPlanner:
    def __init__(self,map):
        self.map = map



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

