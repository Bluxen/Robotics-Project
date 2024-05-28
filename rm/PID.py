from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import pickle
import cv2
from cv2 import aruco
import numpy as np


class pid():
    
    def __init__(self,logger=None, Kp=0.06,Ki=0,Kd=0):
        self.logger=logger
        self.initialize_controller(Kp,Ki,Kd)

    def initialize_controller(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki 
        self.Kd = Kd
        self.last_e = None
        self.sum_e = 0 
        self.dt= 0.01
        
    

    def log(self, str):
        if self.logger is not None:
            self.logger.info(f"{str}")


    def step(self, e):
        """ controller step """
        if(self.last_e is not None):
            derivative = (e - self.last_e) / self.dt
        else:
            #for the first call
            derivative = 0
        self.last_e = e
        self.sum_e += e * self.dt # NEW
        # print(f'Turning of {self.Kp * e + self.Kd * derivative + self.Ki * self.sum_e}')
        return np.clip(self.Kp * e + self.Kd * derivative + self.Ki * self.sum_e, -1.5, +1.5)
    

    
