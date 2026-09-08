import math
import time

#pid class
class pidcontrol:
    def __init__(
        self,
        kp=1.0, ki=0.0, kd=0.0,
        #output limits 
        min_out=-10.0, max_out=10.0,
        # prevent integral windup
        i_max=5.0,
        deadzone=0.01,
        #to check linear or angular
        is_angle=False
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.i_max = i_max
        self.deadzone = deadzone
        self.is_angle = is_angle
        
        #memory for integral accumulateion
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    #functions which calculate the error
    def normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
        
    def take_target(self, target):
        self.target = target
           
    #calculate the error returns the signal
    #takes the target, current state   
    def compute(self, target, current_val, current_time=None):
        error = target - current_val
        if self.is_angle:
            error = self.normalize(error)
                
        # if error less that deadzone output is ok 
        if abs(error) < self.deadzone:
            self.reset()
            return 0.0
            
        
        if current_time is None:
            dt = 0.1
        else:
            if self.prev_time is None:
                dt = 0.1
            else:
                dt = current_time - self.prev_time
            self.prev_time = current_time            
                
        if dt <= 0.0:
            dt = 0.0001
                
        # reset the integral 
        if (error > 0 and self.prev_error < 0) or (error < 0 and self.prev_error > 0):
            self.integral = 0.0
        else:
            self.integral = self.integral + (error * dt)       
                
        #restrict the integral within the limit
        self.integral = max(-self.i_max, min(self.i_max, self.integral))
            
        #calculate the derivateive accroding to (error- prev error) / dt
        der = (error - self.prev_error) / dt
            
        # to find the total output 
        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * der
        out_sig = p + i + d
            
        # restrict the  output 
        out_sig = max(self.min_out, min(self.max_out, out_sig))   
            
        #update the  previous error value
        self.prev_error = error
        return out_sig  
        
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None