import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image

import numpy as np
from numpy import pi
import os
import cv2
from cv_bridge import CvBridge
from random import random
bridge = CvBridge()

from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import sys
import asyncio
import pickle

def rand_round(fr=-pi, to=pi):
    return (random()*(to-fr))+fr

class Calibration(Node):
    CALIBRATION_DATA_FILE = SHARE + 'calibration_data.pickle'
    BOARD_DIM = (4,4)
    BOARD_SIZE = BOARD_DIM[0]*BOARD_DIM[1]
    def __init__(self):
        super().__init__('controller_node')
        self.done = asyncio.Future()

        self.current_camera_msg = None
        self.target_linear_pub = self.create_publisher(Vector3, 'target_linear', 10)
        self.target_angular_pub = self.create_publisher(Vector3, 'target_angular', 10)

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

        self.count = 0
        
    def start(self): 
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.image_callback, 10)
        self.save_timer = self.create_timer(0.3, self.save_camera_msg)

    def save_camera_msg(self):
        if self.current_camera_msg is None: return
        msg = self.current_camera_msg

        x=0.
        y=rand_round(-0.1, 0.1)
        z=0.
        self.target_linear_pub.publish(Vector3(x=x, y=y, z=z))
        x=rand_round(-pi/8, pi/8)
        y=rand_round(-pi/8, pi/8)
        z=rand_round(-pi/3, pi/3)
        self.target_angular_pub.publish(Vector3(x=x, y=y, z=z))

        objp = np.zeros((self.BOARD_SIZE,3), np.float32)
        objp[:,:2] = np.mgrid[0:self.BOARD_DIM[0],0:self.BOARD_DIM[1]].T.reshape(-1,2)
        
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.BOARD_SIZE,3), np.float32)
        objp[:,:2] = np.mgrid[0:self.BOARD_DIM[0],0:self.BOARD_DIM[1]].T.reshape(-1,2)
        
        img = bridge.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, self.BOARD_DIM, None)
        
        # If found, add object points, image points (after refining them)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
            self.imgpoints.append(corners2)
            self.objpoints.append(objp)
            
            self.count += 1
            self.info(self.count)
        if self.count == 5:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, 
                self.imgpoints, 
                gray.shape[::-1], 
                None, None)

            if not ret: 
                self.info("Calibration failed")
                self.done.set_result(1)
                return
            self.info("Finished calibrating")

            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
                
            print("total error: {}".format(mean_error/len(self.objpoints)))

            with open(self.CALIBRATION_DATA_FILE, 'wb') as f:
                pickle.dump([mtx, dist, rvecs, tvecs], f)
            self.info(f"Saved calibration data in {self.CALIBRATION_DATA_FILE}")
            self.done.set_result(1)

    def image_callback(self, msg: Image):
        self.current_camera_msg = msg

    def info(self, *str):
        self.get_logger().info(f"{str}")
    
def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    os.environ["XDG_SESSION_TYPE"] = "xcb"
    
    # Create an instance of your node class
    node = Calibration()
    node.start()
    
    try:
        rclpy.spin_until_future_complete(node, node.done)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()