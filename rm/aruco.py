from cv2 import aruco
import cv2
import numpy as np

from .calibration_data import load_calibration

class Aruco:
    def __init__(self, dictionary_to_use=aruco.DICT_ARUCO_ORIGINAL, size=0.05, logger=None) -> None:
        self.logger = logger
        self.dictionary=aruco.getPredefinedDictionary(dictionary_to_use)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        self.size = size
        self.data = load_calibration()

    def get_aruco_poses(self,corners):
        rvecs, tvecs, objp = aruco.estimatePoseSingleMarkers(corners, self.size, self.data.mtx, self.data.dist)
        return rvecs, tvecs, objp
    
    def detect(self,image):
        image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image=np.asmatrix(image)
        corners, ids, rejected_img_points = self.detector.detectMarkers(image)
        return corners, ids, rejected_img_points
    
    def draw_markers(self, image, markers, ids, rvecs, tvecs, color=(0, 255, 0)):
        aruco.drawDetectedMarkers(image, markers, ids, color)
        for rvec, tvec in zip(rvecs, tvecs):
            cv2.drawFrameAxes(image, self.data.mtx, self.data.dist, rvec, tvec, self.size * 1.5, 2)
        return image
    
    def draw_pose(self, image, rvec, tvec):
        cv2.drawFrameAxes(image, self.data.mtx, self.data.dist, rvec, tvec, self.size * 0.5, 4)