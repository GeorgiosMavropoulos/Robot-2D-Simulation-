"""
PID Controller Module with Anti-Windup Clamping.

This module provides a robust PID (Proportional-Integral-Derivative) controller
equipped with conditional integration (clamping) to prevent integral windup 
during actuator saturation. It includes strict parameter validation at 
initialization and runtime.
"""
import math

class PID:
    
    
    def __init__(self,kp,ki,kd,output_limits):

         """ 
          Uses the helper method _validate_init_args to validate the initialization arguments.
          It validates that arguments are valid int/floats/tuple(float).
          Args:
               kp -> P: Proportional (int/float)
               ki -> I: Integral (int/float)
               kd -> D: Derivative (int/float)
               output_limits:     tuple(float) e.g. (-100.0, 100.0)
               values, or low is greater than or equal to high.
          """   
             
         #valid init arguments
         self._validate_init_args(kp,ki,kd,output_limits)

         self.kp = kp
         self.ki = ki
         self.kd = kd
         self.output_limits = output_limits
         self.integral = 0
         self.previous_error = 0
         


    def _validate_init_args(self,kp,ki,kd,output_limits):
         """
            Validates that arguments during initialization are valid finite numbers. Output must be tuple (e.g. None,None)
            Args:
                 kp -> float/int finite number
                 ki -> float/int finite number
                 kd -> float/int finite number
                 output_limits-> tuple (float) can be None also.

           Raises: 
                  TypeError: kp,ki,kd are not valid float or int numbers,
                             output_limits is valid tuple (float)
                  ValueError: kp,ki,kd are not finite positive numbers (not bool),
                              output_limits is finite numbers not bool
         """     

         if isinstance(kp,bool) or isinstance(ki,bool) or isinstance(kd,bool) or isinstance(output_limits,bool):
              raise TypeError('Kp,ki,kd or output_limits cannot be booleans')
         elif not isinstance(kp,(int,float)) or not isinstance(ki,(int,float)) or not isinstance(kd,(int,float)):
              raise TypeError('Kp,ki,kd must be integers or float numbers.')
         elif not math.isfinite(kp) or not math.isfinite(ki) or not math.isfinite(kd):
              raise ValueError('kp,ki,kd,output_limits must be valid finite numbers')

         #validate output_limits
         if isinstance(output_limits, bool) or not isinstance(output_limits, (tuple, list)) or len(output_limits) != 2:
               raise TypeError('output_limits must be a valid tuple or list of 2 elements.')

         #if output_limits is not None validate that if it's bool,float/int, finite number and the min value is lower than the max
         if output_limits is not None:   
                for x in output_limits:
                    if x is None:
                         continue
                    if isinstance(x,bool):              
                         raise TypeError('Output limits muse be a valid tuple (float,float)')
                    if not isinstance(x,(float,int)):
                         raise TypeError('Output limits cannot be booleans')
                    elif not math.isfinite(x):
                         raise ValueError('Output limits must be valid finite numbers.')
               
         if output_limits[0] > output_limits[1]:
              raise ValueError('The first value of output_limits (min) must be lower than the second value (max).')
          

    def reset(self):
         """Reset error and dt after creating a pid object so the accumulated error will be set to zero"""
        
         self.integral = 0
         self.previous_error = 0
         return self.integral, self.previous_error



    def update(self,error,dt)->float:
         """
        Computing the PID error and applying output limit saturation protection.
        
        Args:
            error -> int/float
            dt -> int/float
          
        Returns:
            Saturated and anti-windup adjusted PID control output

        Raises:
              TypeError:
                        error or dt are booleans,
                        error and dt are not valid integers or float numbers
               ValueError:
                        Error and dt are not finite numbers
                        dt is not greater than zero
        """

         #validate that error is valid, not non-finite number or boolean
         #validate that dt is > 0 and a valid number
         if isinstance(error,bool) or isinstance(dt,bool):
              raise TypeError('Error or dt cannot be booleans')
         elif not isinstance(error,(float,int)) or not isinstance(dt,(float,int)):
              raise TypeError('Error and dt must be valid integer or float numbers')
         elif not math.isfinite(error) or not math.isfinite(dt):
              raise ValueError('Error and dt must be finite numbers')
         elif dt <=0:
              raise ValueError('dt must be greater than zero')


         #store accumulated error's value into a variable in order to return to this value if _PID exceeds output_limit's values
         original_value_accumulated_error =self.integral 
         #calculate proportional
         P = self.kp * error

         #calculate Integral
         self.integral += error * dt
         I = self.integral * self.ki

         #calculate Derivative
         D = (error - self.previous_error) / dt
         d_contribution = D * self.kd

         #update previous error
         self.previous_error = error

         #calculate PID
         pid = P + I + d_contribution

         #validate whether _PID is inside output_limits
                       
         output, saturated = self._clamp(pid)
         if saturated:
          #anti-windup: undo this step's integral increase
          self.integral = original_value_accumulated_error
         return output    




    #helper method to clamp value to output_limits
    def _clamp(self, value)-> tuple[float]:
        """Cut a value to output_limits.

        Args:
            value (float): Raw controller output (P + I + D).

        Returns:
            tuple: (clamped_value, saturated). saturated is True if the
            value was outside the limits and had to be cut; update() uses
            it to decide whether to apply anti-windup.
        """
        low, high = self.output_limits
        if low is not None and value < low:
            return low, True
        if high is not None and value > high:
            return high, True
        return value, False

