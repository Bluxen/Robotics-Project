import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

from robomaster import robot as rm

import tf_transformations
import time

from geometry_msgs.msg import Twist, Pose, Vector3, PointStamped
from robomaster_msgs.msg import GripperState
from robomaster_msgs.action import GripperControl
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image,CameraInfo
import sys

# NEW IMPORTS
import numpy as np
from numpy import pi
import cv2
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from .aruco import Aruco
from .vs import VS

from cv_bridge import CvBridge
import robomaster

class RMNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.vs = VS(logger=self.get_logger())
        self.aruco = Aruco(logger=self.get_logger())
        self.bridge = CvBridge()

        self.vel_pub   = self.create_publisher(Twist,   'cmd_vel',   10)
        self.arm_pub   = self.create_publisher(Vector3, 'cmd_arm',   10)
        self.image_pub = self.create_publisher(Image,   'debug_img', 10)

        self.arm_x = 1.

    def start(self):
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
        self.arm_sub = self.create_subscription(PointStamped, 'arm_position', self.arm_callback, 10)

    def arm_callback(self, msg):
        self.arm_x = msg.point.x

    def move(self, x=0, y=0, z=0, tx=0, ty=0, tz=0):
        self.get_logger().info(f"arm: {self.arm_x}")
        if self.arm_x < 0.158:
            arm_x, body_x = 0.1*x, 0.9*x
        else:
            arm_x, body_x = 0., x
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(body_x), y=float(y)),
            angular=Vector3(z=float(tz))))
        self.arm_pub.publish(Vector3(x=float(arm_x),z=float(z)))

    def mktransform(self, tvec, rvec):
        R, _ = cv2.Rodrigues(rvec)
        Rt = np.hstack((R, tvec.T))
        return np.vstack((Rt, np.array((0, 0, 0, 1))))
    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        
        corners, ids, rejected_img_points = self.aruco.detect(img)

        if ids is not None:
            rvecs, tvecs, objp = self.aruco.get_aruco_poses(corners)
            img = self.aruco.draw_markers(img, corners, ids, rvecs, tvecs)

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            P = self.mktransform(np.matrix(tvec), np.matrix(rvec))
            T = self.mktransform(np.matrix([0., 0.05, 0.15]), np.matrix([0., 0., 0.]))
            M = P @ T
            R = M[:3,:3]
            nrvec, _ = cv2.Rodrigues(R)
            ntvec = M[:3,3]

            theta = nrvec[2]
            # self.get_logger().info(f"{(theta + pi) % pi}")

            self.move(
                x=ntvec[2] *  0.2,
                y=ntvec[0] * -0.4,
                z=ntvec[1] * -0.2,
                tz=theta * -0.2
                )
        else:
            self.move()
            
        # self.get_logger().info(f"{corners}")
        self.image_pub.publish(msg)

def main():
    rclpy.init(args = sys.argv)
    os.environ["XDG_SESSION_TYPE"] = "xcb"
    
    # Create an instance of your node class
    node = RMNode()
    print('starting the controller')
    node.start()

    # Keep processings events until someone manually shuts down the node
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()

if __name__ == '__main__':
    main()