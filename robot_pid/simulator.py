"""
Simulator module
"""
from pid import PID
from planner import AStarPlanner
from robot import Robot
from map import Map
import pygame
import math

class Simulator:
    def __init__(self,grid_map,start_cell,goal_cell):
        self.map = Map(grid_map)
        self.planner = AStarPlanner(self.map)
        self.waypoints = self.generate_waypoints(start_cell,goal_cell)
        self.x,self.y = self.waypoints[0]




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
    

  