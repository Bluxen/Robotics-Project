import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
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
import cv2
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from .aruco import Aruco
from .vs import VS

from cv_bridge import CvBridge

class RMNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.vs = VS(320, 640, logger=self.get_logger())
        self.aruco = Aruco(logger=self.get_logger())
        self.bridge = CvBridge()
        
        self.vel_pub   = self.create_publisher(Twist,   'cmd_vel',   10)
        self.vel_arm   = self.create_publisher(Vector3, 'cmd_arm',   10)
        self.image_pub = self.create_publisher(Image,   'debug_img', 10)

    def start(self):
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)

    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        
        corners, ids, rejected_img_points = self.aruco.detect(img)

        if ids is not None:
            rvecs, tvecs, objp = self.aruco.get_aruco_poses(corners)
            img = self.aruco.draw_markers(img, corners, ids, rvecs, tvecs)
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