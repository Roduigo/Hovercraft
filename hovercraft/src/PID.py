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
            output = max(min(output, self._max_output), self._min_output) #garantir que não vai ultrapassar os limites
        
        if self._max_output is not None and self.Ki != 0:
            self.sum = min(self.sum, self._max_output / (self.Ki * self.sample_time)) # caso ultrapasse o limite superior, retira o integrativo atual
        if self._min_output is not None and self.Ki != 0:
            self.sum = max(self.sum, self._min_output / (self.Ki * self.sample_time)) # caso ultrapasse o limite inferior, retira o integrativo atual

        self.last_error = self.error
        return output