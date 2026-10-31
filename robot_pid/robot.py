"""Kinematic Model in a 2D space.

This module provides the Robot class designed to simulate a moving robot on a
two-dimensional plane (x, y, theta). Using the Forward Euler Integration method,
the robot's state updates over discrete time steps based on linear and angular
velocities. The module features robust type and value verification blocks to
mitigate non-finite inputs or unstable simulation clocks.
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
        TypeError: If the angle is a boolean or not a numeric type (int, float).
        ValueError: If the angle value is not a finite number.
    """

    if  isinstance(a,bool):
        raise TypeError('Angle cannot be a boolean')
    elif not isinstance(a,(int,float)):
        raise TypeError('Angle must be integer or float')

    if not math.isfinite(a):
         raise ValueError('a value must be a finite number')
    
    return (a +math.pi) %(2 * math.pi) - math.pi

class Robot:
    """Represents a mobile robot moving in a 2D kinematic plane."""
    def __init__(self,x,y,theta,linear_velocity,angular_velocity):
        """
        Initializes the Robot with states and velocities.
        Args:
            x (int | float): Initial position on the X-axis.
            y (int | float): Initial position on the Y-axis.
            theta (int | float): Heading orientation angle in radians.
            linear_velocity (int | float): Initial forward linear speed.
            angular_velocity (int | float): Initial steering angular speed.

        Uses _validate_init_args method to validate initialization arguments

        
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
        Validates constructor properties against incorrect types or non-finite values.

        Maps state arguments inside a validation loop to verify safe parameters
        before configuring active object states.

        Args:
            x (int | float): Position coordinate on the X-axis.
            y (int | float): Position coordinate on the Y-axis.
            theta (int | float): Heading orientation angle in radians.
            linear_velocity (int | float): Forward linear speed.
            angular_velocity (int | float): Steering angular speed.

        Raises:
            TypeError: If any parameter is a boolean or not a valid number.
            ValueError: If any parameter represents an infinite value or NaN.
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

    def step(self,dt)-> tuple[float]:
        """
        Propagates the robot state forward by a discrete time step.

        Uses Forward Euler Integration along the active velocity vector fields 
        to track spatial mutations across simulated time frame segments.

        Args:
            dt (int | float): Delta time segment duration (must be > 0).

        Returns:
            tuple[float, float, float]: The updated state profile (x, y, theta).

        Raises:
            TypeError: If the time step is a boolean or missing a numeric type.
            ValueError: If the time step is non-finite, negative, or zero.
        """
        
        #raise an error if dt is a boolean
        if isinstance(dt,bool):
            raise TypeError('dt cannot be a boolean')
        elif not isinstance(dt,(int,float)):
            raise TypeError('dt must be a float or integer')
        elif not math.isfinite(dt):
            raise ValueError('dt must be a finite number')

        elif dt <= 0:
           raise ValueError('dt must be greater than zero')

        #calcuate robot's new position
        x_new_pos = self.x + self.linear_velocity * math.cos(self.theta) * dt
        y_new_pos = self.y + self.linear_velocity * math.sin(self.theta) * dt

        
        new_theta = angle_wrap(self.theta + self.angular_velocity * dt )

        #update x,y,theta
        self.x = x_new_pos
        self.y = y_new_pos
        self.theta = new_theta

        return (self.x,self.y,self.theta)
                        
