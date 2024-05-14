from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import pickle
import cv2
import numpy as np

from cv_bridge import CvBridge
from .calibration_data import load_calibration

def mktr(x, y, z): # pure translation
    ret = np.eye(4)
    ret[0:3,3] = x,y,z
    return ret

class VS():
    def __init__(self, width, height, FOV=120, logger=None):
        self.camera_width = width
        self.camera_height = height
        self.FOV = FOV
        self.logger=logger
        self.data = load_calibration()

    def initialize_controller(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki 
        self.Kd = Kd
        self.last_e = None
        self.sum_e = 0 
        self.dt= 0.1
        
    def get_corners(self, rvecs, tvecs, objp):
        corners = []
        for rvec, tvec, objc in zip(rvecs, tvecs, objp):
            self.log(tvec)
            self.log(tvecs)
            T = mktr(*tvec)
            R = cv2.Rodrigues(rvec)
            marker_transform = T @ R 
            corners.append([marker_transform @ obj for obj in objc])
        self.log(corners)
        return corners
    
    def log(self, str):
        if self.logger is not None:
            self.logger.info(f"{str}")