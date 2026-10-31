"""
Kinematic Model in a 2D space

-------------------------------------------------------------
This module contains the Robot class in order to simulate a moving robot on space (x,y,theta).
Using Euler method (Forward Euler Integration), robot's state gets updated in discrete time steps (dt)
based on current linear (v) and angular (omega) speed. 
It contains strict error handling mechanisms to avoid non-finite values (NaN/Inf) and negative dt ensuinrg stability during the simulation.
"""
import math
#utility function
def angle_wrap(a)->float:
    """Wraps an angular value to the interval [-pi, pi].

    This utility ensures that orientation angles stay within a standardized 
    range, preventing arithmetic overflow and keeping trigonometric behaviors 
    consistent.

    Args:
        a (float): The input angle in radians.

    Returns:
        float: The wrapped angle bounded within [-pi, pi].

    Raises:
           ValueError: 
                      if angle is not a finite number

           TypeError:
                     if angle is not a float or int
                     if angle is boolean
    """

    if  isinstance(a,bool):
        raise TypeError('Angle cannot be a boolean')
    elif not isinstance(a,(int,float)):
        raise TypeError('Angle must be integer or float')

    if not math.isfinite(a):
         raise ValueError('a value must be a finite number')
    
    return (a +math.pi) %(2 * math.pi) - math.pi

class Robot:
    def __init__(self,x,y,theta,linear_velocity,angular_velocity):
        """
        Args: 
            x-> position on X axis (float)
            y -> position on Y axis (float)
            theta -> radians
            linear_velocity-> numerical value
            angular_velocity -> numerical value

        
        """
        #validate arguments
        self._validate_init_args(x,y,theta,linear_velocity,angular_velocity)
        self.x = x
        self.y = y
        self.theta = angle_wrap(theta)
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity


    def _validate_init_args(self,x,y,theta,linear_velocity,angular_velocity):
        """
         Validate constructor's arguments

          Args:
                x -> float/int
                y-> float/int
                theta -> radians
                linear_velocity -> numerical value
                angular_velocity -> numerican value
        """

        variables_to_check = {
                'x': x,
                'y': y,
                'theta': theta,
                'linear_velocity': linear_velocity,
                'angular_velocity': angular_velocity
            }
        for name,value in variables_to_check.items():
            if isinstance(value,bool):
              raise TypeError(f'{name} cannot be a boolean')

            elif not isinstance(value,(float,int)):
                raise TypeError(f'{name} must be float or integer')
            
            elif not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
            
            

