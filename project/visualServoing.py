from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import pickle
import cv2
from cv2 import aruco
import numpy as np
from.arucoHelperclass import arucoHelper


class VS():
    CALIBRATION_DATA_FILE = SHARE + 'calibration_data.pickle'
    def __init__(self, width, height, FOV=120, logger=None, Kp=0.6,Ki=0,Kd=0):
        self.camera_width = width
        self.camera_height = height
        self.FOV = FOV
        self.logger=logger
        
        with open(self.CALIBRATION_DATA_FILE, 'rb') as f:
            self.mtx, self.dist, self.rvecs, self.tvecs = pickle.load(f)

        self.initialize_controller(Kp,Ki,Kd)

    def initialize_controller(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki 
        self.Kd = Kd
        self.last_e = None
        self.sum_e = 0 
        self.dt= 0.1
        
    def draw_pose(self, image, corners, ids, size):
        rvecs, tvecs, objp = self.get_poses(corners, size)
        self.log(f'The returning rotation and traslation of the aruco:{rvecs},{tvecs},{objp}')
        for i in range(len(ids)):
            cv2.drawFrameAxes(image, self.mtx, self.dist, rvecs[i], tvecs[i], size * 1.5, 2)

        return image
    

    def draw_calibration_tuning(self, image, corners, ids, size, camera_imx, camera_dimy, arucoH:arucoHelper):
        rvecs, tvecs, objp = self.get_poses(corners, size)
        self.log(f'The returning rotation and traslation of the aruco:{rvecs},{tvecs},{objp}')
        for i in range(len(ids)):
            #center of aruco
            cv2.drawFrameAxes(image, self.mtx, self.dist, rvecs[i], tvecs[i], size * 1.5, 2)
            cv2.circle(image, centerOfCircle=arucoH.getArUcoCentre(corners[i]), radius=5, color=(255,0,0), thickness=-1)

            #center of the image
            centerOfCircle=(camera_imx/2.0, camera_dimy/2.0)
            cv2.circle(image, centerOfCircle=centerOfCircle, radius=5, color=(0,255,0), thickness=-1)
        return image


    def get_poses(self,corners,size):
        rvecs, tvecs, objp = aruco.estimatePoseSingleMarkers(corners, size, self.mtx, self.dist)
        return rvecs, tvecs, objp
        # cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        
        # if(estimatePose) {
        # for(unsigned int i = 0; i < ids.size(); i++)
        # cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
        # }

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
    

    def decideTurning(self, aruco_centre_x, coords_x):
        if np.isclose(aruco_centre_x,coords_x, 0.01):
            return 'stop'
        elif aruco_centre_x>coords_x:
            return 'left'
        return 'right'

        pass