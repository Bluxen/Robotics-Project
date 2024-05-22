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
import time
import numpy as np
from numpy import pi
import cv2
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from .aruco import Aruco
from .vs import VS
from asyncio import Future

from cv_bridge import CvBridge
import robomaster
from .robotStates import Rstates

class RMNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.vs = VS(logger=self.get_logger())
        self.aruco = Aruco(logger=self.get_logger())
        self.bridge = CvBridge()
        self.done = Future()

        self.vel_pub   = self.create_publisher(Twist,   'cmd_vel',   10)
        self.arm_pub   = self.create_publisher(Vector3, 'cmd_arm',   10)
        self.image_pub = self.create_publisher(Image,   'debug_img', 10)

        self.arm_x = 1.
        self.seen = None
        self.v = None
        self.state=Rstates.INITIAL

    def start(self):
        self.get_logger().info('In start')
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
        self.arm_sub = self.create_subscription(PointStamped, 'arm_position', self.arm_callback, 10)
        if self.state==Rstates.INITIAL:
            self.state=Rstates.SEARCHING
            self.get_logger().info('Searching an Aruco')
        if self.state==Rstates.SEARCHING:
            self.arm_pub.publish(Vector3(x=float(0.0),z=float(-0.1)))
            self.get_logger().info('Start spotting')
            self.searcher = self.create_timer(1/60, self.search_aruco)
        else:
            self.get_logger().info('DONE THIS PART')
            self.timer = self.create_timer(1/60, self.timer_callback)


    def search_aruco(self):
        self.get_logger().info('in method')
        if self.state!=Rstates.SEARCHING:
            self.get_logger().info("ARUCO SPOTTED")
            self.state=Rstates.ALIGNING
            self.move(0.0, 0.0, 0.0)
            self.destroy_timer(self.searcher)
            return
        self.get_logger().info('NOT SPOTTED')
        if True:
            z = 0.8
            self.move(0.0, 0.0, z)
            return
        
    def move(self, x_linear = 0.0, y_linear = 0.0, z_angular = 0.0):
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(x_linear), y=float(y_linear)),
            angular=Vector3(z=float(z_angular))))
        
    def arm_callback(self, msg):
        self.arm_x = msg.point.x

    def timer_callback(self):
        if self.v is None: return
        t = time.time()
        tvec, theta = self.v
        # Proportional controller
        #               P
        x  = tvec[2] *  0.1
        y  = tvec[0] * -0.4
        z  = tvec[1] * -0.1
        tz = theta   * -0.1
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

    def move(self, x=0, y=0, z=0, tx=0, ty=0, tz=0):
        if self.arm_x < 0.180 and x>0:
            arm_x, body_x = 0.4*x, 0.
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
    
    def img_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg)
        
        corners, ids, rejected_img_points = self.aruco.detect(img)

        if ids is not None:
            self.state=Rstates.ALIGNING
            self.seen = time.time()
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
            self.v = (ntvec, theta)
            speed = (np.abs(ntvec).sum() + abs(theta)) / 4
            if speed < 0.02:
                self.get_logger().info("Done")
                self.done.set_result(1)
            
        self.image_pub.publish(msg)

    def stop(self): self.move()

def main():
    rclpy.init(args = sys.argv)
    os.environ["XDG_SESSION_TYPE"] = "xcb"
    
    # Create an instance of your node class
    node = RMNode()
    node.start()

    # Keep processings events until someone manually shuts down the node
    try: rclpy.spin_until_future_complete(node, node.done)
    except KeyboardInterrupt: pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()

if __name__ == '__main__':
    main()