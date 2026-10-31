"""
PID implementation


"""
import math

class PID:
    
    
    def __init__(self,kp,ki,kd,output_limits):

         """ 
           Uses the helper method _validate_int_args to validate the initialization arguments. It validates that arguments are valid int/floats/tuple(float).
         Args:

            kp -> P: Proportional (int/float)
            ki -> I: Integral (int/float)
            kd -> D: Derivate (int/float)
            output_limits:     tuple(float) e.g. None None


         
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
         for x in output_limits:
              if isinstance(x,bool):              
                   raise TypeError('Output limits muse be a valid tuple (float,float)')
              if not isinstance(x,(float)):
                   raise TypeError('Output limits cannot be booleans')
              elif not math.isfinite(x):
                   raise ValueError('Output limits must be valid finite numbers.')
              
         if output_limits[0] > output_limits[1]:
               raise ValueError('The first value of output_limits (min) must be lower than the second value (max).')
         

    def reset(self)->float:
         """Reset error and dt after creating a pid object so the accumulated error will be set to zero"""
        
         self.integral = 0
         self.previous_error = 0
         return self.integral, self.previous_error