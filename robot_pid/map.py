"""
2D Grid Map Representation Module.

This module provides the Map class used to parse, store, and validate a grid-based 
occupancy environment. It defines obstacle and free space properties to assist 
robotic path planning and localization scripts across discrete grid spaces.
"""
import math
class Map:
    """Represents a 2D occupancy grid environment for motion planning."""
    def __init__(self,grid_map,cell_size=0.5):
        """
        Initializes the occupancy grid map with safe dimensions.

        Passes parameters directly into the internal  method 
        _validate_init_args to ensure grid stability before allocating active 
        state memory and properties.       

        Args:
            grid_map (list[list[int]]): A two-dimensional nested list containing 
                integers where 0 represents free space and 1 represents an obstacle.
            cell_size (int | float, optional): The physical metric dimension of a 
                single grid cell edge. Defaults to 0.5.
        """
        
        #valid initialization arguments
        self._validate_init_args(grid_map,cell_size)

        self.grid_map = grid_map
        self.cell_size = cell_size
        #create rows and cols
        self.rows = len(grid_map)
        self.cols = len(grid_map[0])



    def _validate_init_args(self,grid_map,cell_size):
        """
        Validates structural types, sizing dimensions, and elements inside the grid map.

        Checks both the environmental resolution scale and the multi-dimensional 
        matrix to enforce rectangular dimensions, eliminate empty structural 
        definitions, and ensure internal cell indices consist solely of literal 
        integer binary flags (0 and 1).

        Args:
            grid_map (list[list[int]]): The multi-dimensional nested array under 
                structural verification.
            cell_size (int | float): The physical metric dimension of a single cell.

        Raises:
            TypeError: If cell_size is a boolean, not a number, or if grid_map is not 
                a list, its rows are not lists, or its cell values are booleans/non-integers.
            ValueError: If cell_size is non-finite, negative, or zero; or if grid_map 
                is empty, any nested row is empty, row dimensions vary, or cells contain 
                values other than 0 or 1.
        """        

        #validate cell_size
        if isinstance(cell_size,bool):
              raise TypeError('cell_size cannot be a boolean')
        elif not isinstance(cell_size,(int,float)):
            raise TypeError('cell_size must be an integer or a float')
       
        elif cell_size <=0:
            raise ValueError('cell_size must be greater than zero')
        elif not math.isfinite(cell_size):
            raise ValueError('cell_size must be a finite number')

        if not isinstance(grid_map,list):
            raise TypeError('grid_map must be a list')
        elif not grid_map:
            raise ValueError('grid_map cannot be empty')

        expected_cols = None

        #validate that grid map contains a nested list an all items are integers
        for row in grid_map:
            if not isinstance(row,list):
                raise TypeError('grid_map items must be lists')
          
            elif not row:
                raise ValueError('Grid rows cannot be empty')
            
            
            #compare row's length with col's expected_cols 
            if expected_cols is None:             
                expected_cols = len(grid_map[0]) #take  numbers of columns
            
            if len(row) != expected_cols:
                    raise ValueError('All rows must have the same number of columns')

            #validate that columns are being built from 1 and 0
            for col in row:
                    if isinstance(col,bool):
                        raise TypeError('Columns cannot be composed of booleans')
                    elif not isinstance(col,int):
                        raise TypeError('Columns must be composed of integers')
                    elif not (0 <= col <= 1):
                        raise ValueError('Columns must contain values in range of 0-1')

    
    def _is_inbound(self,row,col)->bool:
        """
           Return Falseif row and col are out of map's boundaries, otherwise True

           Args:
                row (int): The row index of the cell in the grid map.
                col (int): The column index of the cell in the grid map.

           Return:
                   True if row,col are inside the grid
                   Otherwise False
        """
        
        #return True if row and col are outside the map
        return  (0 <= row < self.rows and 0 <= col < self.cols)
          


    def cell_to_world(self,row,col)-> tuple[float,float]:
        """
        Converts a cell index into world x, y coordinates.

        Calculates the physical metric coordinates of the center of a specific
        grid cell based on the map's cell size.

        Args:
            row (int): The row index of the cell in the grid map.
            col (int): The column index of the cell in the grid map.

        Returns:
            tuple[float, float]: A tuple containing the (x, y) world coordinates.

        Raises:
            TypeError: If row or col is a boolean or not an integer.
            ValueError: If the cell index is outside the grid map boundaries.
        """
        

        #validate that row and col are float numbers
        if isinstance(row,bool) or isinstance(col,bool):
                    raise TypeError('row and col cannot be booleans')
        elif not isinstance(row,int) or not isinstance(col,int):
            raise TypeError('row and col must be integer numbers')

        #validate that cell is inside boundaries
        if not (0 <= row < self.rows) or not (0 <= col < self.cols):
             raise ValueError('Cell is out of boundaries')

        #compute world coordinate. use 0.5 to return cell's center
        x = (col +0.5) * self.cell_size
        y = (self.rows - row - 0.5) * self.cell_size

        return x,y


    def world_to_cell(self,x,y)-> tuple[int,int]:
         """
         Converts world x, y coordinates into grid cell coordinates.

        Identifies which discrete grid cell contains the continuous world 
        point to help verify the robot's state and proximity to obstacles.

        Args:
            x (int | float): The continuous x coordinate in the world space.
            y (int | float): The continuous y coordinate in the world space.

        Returns:
            tuple[int, int]: A tuple containing the (row, col) cell indices.

        Raises:
            TypeError: If x or y is a boolean, or not a float/integer.
            ValueError: If x or y is a non-finite number.
         """
         world_coordinates = [('x', x), ('y', y)]
         #raise an error if x or y are booleans
         for name,value in world_coordinates:
              if isinstance(value,bool):
                   raise TypeError(f'{value} cannot be a boolean')
              elif not isinstance(value,(float,int)):
                   raise TypeError(f'{name} must be a float or integer')
              elif not math.isfinite(value):
                   raise ValueError(f'{name} must be a finite number')
              
      
         #convert x,y into cell. use math.floor() to round the result into the smaller closer value
         _col = x/self.cell_size
         col = math.floor(_col)
         row = self.rows - 1 - math.floor(y / self.cell_size)

         #raise an error if cell it out of boundaries
         if not self._is_inbound(row,col):
                       raise ValueError('Cell is out of boundaries')
         cell = row,col
       
         return cell


    def _is_free_cell(self,cell)-> bool:
         """
         Checks if a specific grid cell is free (obstacle-free) for the robot.

        Args:
            cell (tuple[int, int]): A tuple containing the (row, col) indices.

        Returns:
            bool: True if the cell is free (contains 0), False if it is occupied (contains 1).

        Raises:
            TypeError: If the cell is a boolean, not a tuple, not exactly 
                2 elements, or if its elements are not integers.
            ValueError: If the cell indices are out of the grid boundaries.
        """

         #validate whether cell is a valid tuple or not
         if isinstance(cell,bool):
              raise TypeError('Cell cannot be a boolean')
         elif not isinstance(cell, tuple) or len(cell) != 2:
              raise TypeError('Cell must be a tuple[int,int]')

         #unpack tuple to see if they are inside boundaries of the grid
         row,col = cell
         
         if isinstance(row, bool) or isinstance(col, bool):
            raise TypeError('Cell elements cannot be booleans')
         elif not isinstance(row, int) or not isinstance(col, int):
            raise TypeError('Cell elements must be integers')

         #raise value error if row or col are outside of map's boundaries
         if not self._is_inbound(row,col):
              raise ValueError('row and col must be inside the grid')


         return self.grid_map[row][col] == 0
   



    def neighbors(self,cell) -> list[tuple]:
            """Return the free 4-connected neighbours of a cell.
    
            Neighbours are checked in the order right, left, down, up
            (row +1 is down, row -1 is up).
    
            Args:
                cell (tuple): (row, col) indices.
    
            Returns:
                list[tuple]: (row, col) of every free neighbour; cells outside
                the map or holding an obstacle are left out.
    
            Raises:
                TypeError: If cell is not a pair of integers.
            """
    
            #raise a type error if cell is not a tuple with 2 values
            if not isinstance(cell, tuple) or len(cell) != 2:
              raise TypeError('Cell must be a tuple[int,int]')

           
            
    
            #unpack cell to row and col
            row,col = cell
            #validate row and col are integers and not booleans
            if isinstance(row,bool) or isinstance(col,bool):
                 raise TypeError('row and col cannot be booleans')
            
            elif not isinstance(row,int) or not isinstance(col,int):
                 raise TypeError('row and col must be valid integers')
            
        
            elif not self._is_inbound(row,col):
                      raise ValueError('Cell is outside of boundaries')
    
            #create a list to store the valid blocks the algorithm will find
            neighbors = [] 
    
            #the following list holds the direction's key. each pair represents a move. 
            # (0, 1)->right (same row next column), 
            # (0,-1)->left(same row, previous column),
            ## (1, 0)->(next row, same column), (-1,0)->(previous row, same column)
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0)] 

    
            #loop through the list and for each direction create the neighbor cell adding the change in the row and col
            for direction in directions:
                r,c = row + direction[0], col + direction[1]

                #raise an error if row,col are outside the grid
                if self._is_inbound(r,c):                     
                 #if method is free accepts the cell add it to the list (neighbors)
                 if self._is_free_cell((r, c)):
                    #append the coordinates to the list
                    neighbors.append((r, c)) 
    
            return neighbors



grid = [
    [0, 0, 1],
    [0, 0, 0],
    [1, 0, 0]
]

m = Map(grid)

print(m.neighbors((1, 1)))
print(m.neighbors((0, 0)))
         




        


            