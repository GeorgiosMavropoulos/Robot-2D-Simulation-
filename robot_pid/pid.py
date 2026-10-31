"""
PID implementation


"""
import math

class PID:
    
    
    def __init__(self,kp,ki,kd,output_limits):

         self.kp = kp
         self.ki = ki
         self.kd = kd
         self.output_limits = output_limits
         self.integral = 0
         self.previous_error = 0
         



    def reset(self,integral,previous_error)->float:
         """Reset error and dt after creating a pid object so the accumulated error will be set to zero
            
            Args: 
                 integral: int/float
                 previous_error: int/float
            
                 
            Returns:
                     integral: float (zero)
                     previous_error: float (zero)

            Raises:
                   ValueError: integral or previous_error are negative or not float/int numbers,
                               integral or previous_error are not finite numbers
         
         
         """

         #validate that error and dt values won't be negative
         if not isinstance(integral,(int, float)) or not isinstance(previous_error,(int, float)) or previous_error <0 or integral <0:
              raise ValueError('Integral and previous_error must be positive valid numbers')
         elif not math.isfinite(integral) or not math.isfinite(previous_error):
              raise ValueError('Integral and previous_error must be finite numbers')

        
         integral = 0
         previous_error = 0
         return integral,previous_error