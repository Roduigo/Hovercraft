#!/usr/bin/env python3
# Rodrigo Kurosawa
import rospy
import numpy as np
import matplotlib.pyplot as plt

class PID:
    def __init__(self,
        Kp=1,
        Ki=0.0,
        Kd=0.0,
        sample_time = 1/30, #rostopic hz topico
        _min_output = None,
        _max_output = None
    ):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        
        self._min_output = _min_output   
        self._max_output = _max_output     
        self.sample_time = sample_time
        self.sum = 0
        self.error = 0
        self.last_error = 0
        
    def compute(self, error):
        self.error = error
        
        P = self.Kp * self.error
        self.sum += self.error
        I = self.Ki * self.sum * self.sample_time
        D = self.Kd * (self.error - self.last_error)/self.sample_time
        output = P+I+D
        
        self.last_error = self.error
        #para evitar o integral windup
        if self._max_output is not None and self._min_output is not None:
            if(output < self._min_output):
                self.sum = 0
                return output
            elif(output > self._max_output):
                self.sum += self.error
                return output
            
        return output