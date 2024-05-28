import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import time

from geometry_msgs.msg import Twist, Quaternion, Vector3, PointStamped
from robomaster_msgs.msg import GripperState
from robomaster_msgs.action import GripperControl
from sensor_msgs.msg import Image
import sys
import time
import numpy as np
import cv2
import os
from .aruco import Aruco
from .state import State

from .state_move_forward import MoveForward

from cv_bridge import CvBridge

import math
 
def euler_from_quaternion(x, y, z, w):
    """
    Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/

    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

def mktransform(tvec, rvec):
    R, _ = cv2.Rodrigues(rvec)
    Rt = np.hstack((R, tvec.T))
    return np.vstack((Rt, np.array((0, 0, 0, 1))))

class Align(State):

    def __init__(self, target_id):
        super().__init__()
        self.target_id = target_id

    v = None
    def init(self):
        self.bridge = CvBridge()
        self.aruco = Aruco(logger=self.get_logger())
        self.timer = self.create_timer(1/60, self.timer_callback)
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
        self.ctg_linear_sub  = self.create_subscription(Vector3,    'ctg_linear',  self.ctg_linear_callback, 10)
        self.ctg_angular_sub = self.create_subscription(Quaternion, 'ctg_angular', self.ctg_angular_callback, 10)
        self.roll_linear, self.roll_angular = [], []
        
    def timer_callback(self):
        if self.v is None: return
        t = time.time()
        tvec, theta = self.v
        # Proportional controller
        #               P
        x  = tvec[2] *  0.1
        y  = tvec[0] * -0.4
        z  = tvec[1] * -0.1
        tz = theta   * -0.2
        if self.seen is not None and self.seen + 0.5 > t:
            self.move(x=x, y=y, z=z, tz=tz)
        else:
            alpha = np.interp(t, [self.seen, self.seen+3], [1, 0])
            beta = 1-alpha
            self.move(
                x=alpha*x + beta*-0.2,
                y=alpha*y,
                z=alpha*z + beta*-0.1,
                tz=alpha*tz
            )
    
    def img_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg)
        
        corners, ids, rejected_img_points = self.aruco.detect(img)

        if ids is not None and self.target_id in ids:
            self.seen = time.time()
            rvecs, tvecs, objp = self.aruco.get_aruco_poses(corners)
            img = self.aruco.draw_markers(img, corners, ids, rvecs, tvecs)
            
            idx = ids.tolist().index([self.target_id])
            rvec = rvecs[idx][0]
            tvec = tvecs[idx][0]

            P = mktransform(np.matrix(tvec), np.matrix(rvec))
            T = mktransform(np.matrix([0., 0.05, 0.32]), np.matrix([0., 0., 0.]))
            M = P @ T
            # self.get_logger().info(f"{self.ctg_linear}, {self.ctg_angular}")
            nrvec, _ = cv2.Rodrigues(M[:3,:3])
            ntvec = M[:3,3]
            self.aruco.draw_pose(img, nrvec, ntvec)

            if self.ctg_linear is not None and self.ctg_angular is not None:
                ctg = mktransform(self.ctg_linear, self.ctg_angular)
                M = M @ ctg

            R = M[:3,:3]
            nrvec, _ = cv2.Rodrigues(R)
            ntvec = M[:3,3]

            self.roll_linear.append(ntvec)
            self.roll_angular.append(nrvec)
            self.roll_linear  = self.roll_linear [max(len(self.roll_linear )-30,0):]
            self.roll_angular = self.roll_angular[max(len(self.roll_angular)-30,0):]
            ntvec = (sum(self.roll_linear) /len(self.roll_linear)) 
            nrvec = (sum(self.roll_angular)/len(self.roll_angular))
            self.get_logger().info(f"{ntvec}, {nrvec}")
            
            # nrvec = self
            theta = nrvec[2]
            self.v = (ntvec, theta)
            speed = (np.abs(ntvec).sum() + abs(theta)) / 4

            self.aruco.draw_pose(img, nrvec, ntvec)
            if speed < 0.01:
                self.switch_state(MoveForward())
            
        self.image_pub.publish(msg)

    def ctg_linear_callback(self, msg: Vector3):
        # self.get_logger().info(f"{msg}")
        self.ctg_linear = np.matrix([0., msg.x, msg.z])

    def ctg_angular_callback(self, msg: Quaternion):
        qx, qy, qz, qw = msg.x, msg.y, msg.z, msg.w
        x, y, z = euler_from_quaternion(qx, qy, qz, qw)
        self.ctg_angular = np.matrix([y, 0., 0.])