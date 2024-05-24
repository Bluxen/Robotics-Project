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
from sensor_msgs.msg import Range, Image, CameraInfo, JointState
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
        self.gap = ActionClient(self, GripperControl, 'gripper')
        self.arm_x = 1.
        self.seen = None
        self.v = None
        self.state=Rstates.INITIAL

    def start(self):
        # self.get_logger().info(f'In start {self.state}')
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
        self.arm_sub = self.create_subscription(PointStamped, 'arm_position', self.arm_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        if self.state==Rstates.INITIAL:
            # return
            self.state=Rstates.SEARCHING
            # self.get_logger().info('Searching an Aruco')
        if self.state==Rstates.SEARCHING:
            self.arm_pub.publish(Vector3(x=float(0.0),z=float(-0.1)))
            # self.get_logger().info('Start spotting')
            self.searcher = self.create_timer(1/60, self.search_aruco)
        elif self.state==Rstates.ALIGNING:
            self.timer = self.create_timer(1/60, self.timer_callback)
        elif self.state==Rstates.MOVING_FORWARD:
            self.t = time.time()
            self.timer_fw = self.create_timer(1/60, self.move_forward)
        elif self.state==Rstates.GRAB:
            # self.grabber = self.create_timer(1/60, self.grab)
            self.grab()
        else:
            self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0.0)),
            angular=Vector3(z=float(0.0))))
            self.arm_pub.publish(Vector3(x=float(-0.1),z=float(0.2)))
            # self.get_logger().info('DONE THIS PART')
            self.done.set_result(1)

    def grab(self):
        self.arm_pub.publish(Vector3(x=float(0.3),z=float(0.0)))

        # self.get_logger().info('GRAB GRAB GRAB')
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0.0)),
            angular=Vector3(z=float(0.0))))
        self.t = time.time()
        self.close_gripper()
        

        self.arm_pub.publish(Vector3(x=float(0.0),z=float(0.5)))
        self.state=Rstates.DONE
        # self.destroy_timer(self.grabber)
        self.start()

    def close_gripper(self):
        # self.get_logger().info("Closing gripper")
        # self.get_logger().info('in method')
        result=None
        # while result is None or (result is not None and not result.done()):
            # self.get_logger().info('waiting')
        future=self.gap.send_goal_async(GripperControl.Goal(target_state=GripperState.CLOSE))
        try: rclpy.spin_until_future_complete(self, future,timeout_sec=0.5)
        except KeyboardInterrupt: pass
        # if (result is not None and result.done()):
        #     self.get_logger().info('GRABBEDDDDDD!!!!')
        # else:
        #     self.get_logger().info('NOT GRABBEDDDDDD!!!!')
        self.state=Rstates.DONE



    def search_aruco(self):
        # self.get_logger().info('in method')
        if self.state!=Rstates.SEARCHING:
            # self.get_logger().info("ARUCO SPOTTED")
            self.state=Rstates.ALIGNING
            self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0.0)),
            angular=Vector3(z=float(0.0))))
            self.destroy_timer(self.searcher)
            self.start()
            return
        # self.get_logger().info('NOT SPOTTED')
        if True:
            z = 0.2
            self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0.0)),
            angular=Vector3(z=float(z))))
            return

        
    def arm_callback(self, msg):
        self.arm_x = msg.point.x

    def move_forward(self):
        self.get_logger().info("I'm moving forward")
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.15), y=float(0)),
            angular=Vector3(z=float(0))))
        if self.t+0.25 <= time.time():
            self.get_logger().info("I am done moving forward")
            self.destroy_timer(self.move_forward)
            self.state=Rstates.GRAB
            self.move(0.0,0.0,0.0)
            self.start()
        

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
        if self.arm_x < 0.178 and x>0:
            arm_x, body_x = 0.4*x, 0.6*x
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
            T = self.mktransform(np.matrix([0., 0.15, 0.30]), np.matrix([0., 0., 0.]))
            M = P @ T
            R = M[:3,:3]
            nrvec, _ = cv2.Rodrigues(R)
            ntvec = M[:3,3]

            theta = nrvec[2]
            self.v = (ntvec, theta)
            speed = (np.abs(ntvec).sum() + abs(theta)) / 4
            if speed < 0.025:
                self.state=Rstates.MOVING_FORWARD
                self.destroy_timer(self.timer)
                self.start()
            
        self.image_pub.publish(msg)

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f"{msg.name}")
        joints = ['RM/arm_1_joint', 'RM/arm_2_joint', 'RM/rod_joint', 'RM/rod_1_joint']
        names, poses = msg.name, msg.position
        idxs = [names.index(joint) for joint in joints]
        thes = [poses[idx] for idx in idxs]
        self.get_logger().info(f"{[f'{name}: {theta}' for name, theta in zip(joints, thes)]}")

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