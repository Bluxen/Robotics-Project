# NEW IMPORTS
import PIL
import numpy as np
import cv2
import os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl

class arucoHelper():
    def __init__(self, dictionary_to_use=aruco.DICT_ARUCO_ORIGINAL, logger=None) -> None:
        self.logger = logger
        self.dictionary=cv2.aruco.getPredefinedDictionary(dictionary_to_use)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.tolerance_centre=0.5

    def log(self, str):
        if self.logger is not None:
            self.logger.info(f"{str}")
    
    def getArucoPosition(self,image):
        '''
        This method gets an image as a python array and returns a boolean which states whether the arUco is present or not
        '''
        image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY )
        image=np.asmatrix(image)
        self.log(image)
        corners, ids, rejected_img_points = self.detector.detectMarkers(image)
        return corners, ids, rejected_img_points
    
    def __getArUcoCentre(self, aruco_id, corners):
        # get the centre x value
        topL, topR, bottomR, bottomL= corners
        # topL, topR, bottomR, bottomL=tuple(topL), tuple(topR), tuple(bottomR), tuple(bottomL)
        centre_x=(((topL[0]+ topR[0])/2.0)+(bottomR[0]+ bottomL[0])/2.0)/2.0
        centre_y=(((topL[0]+ bottomL[0])/2.0)+(bottomR[0]+ topR[0])/2.0)/2.0
        return [centre_x,centre_y]

    def getAllArUcoCenter(self, ids, corners):
        centres=[]
        for i,id in enumerate(ids):
            coners_current_ArUco=corners[i]
            centre=self.__getArUcoCentre(id,coners_current_ArUco)
            centres.append(centre)
        return centres
    
    def getsidesLength(self, corners):
        topL, topR, bottomR, bottomL= corners
        len_left = np.linalg.norm(np.array(topL)-np.array(bottomL))
        len_right = np.linalg.norm(np.array(topR)-np.array(bottomR))
        return len_left,len_right
        
    def decideDirection(self,corners):
        len_left, len_right=self.getsidesLength(corners)
        if len_left-len_right<self.tolerance_centre:
            return 'stop'
        elif len_left>len_right:
            return 'right'
        return 'left'

    def drawImage(self, image, squares):
        for square in squares:
            corners = square[0]
            # draw the bounding box of the ArUCo detection
            topLeft,topRight,bottomRight,bottomLeft=[np.array(corner, dtype=int) for corner in corners]
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        # # draw the ArUco marker ID on the image
        # cv2.putText(image, str(markerID),
        #     (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
        #     0.5, (0, 255, 0), 2)
        # print("[INFO] ArUco marker ID: {}".format(markerID))

        # show the output image
        cv2.imshow("Image", image)
        cv2.waitKey(0)