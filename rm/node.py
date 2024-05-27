import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

from robomaster import robot as rm

import tf_transformations
import time

from geometry_msgs.msg import Twist, Quaternion, Vector3, PointStamped
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

import math
from .PID import pid
 
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
        self.ctg_linear, self.ctg_angular = None, None
        self.targetaruco=55
        self.on_the_ground=False
        self.pid=pid(self.get_logger())

    def start(self):
        self.get_logger().info(f'In start {self.state}')
        self.image_sub = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
        self.arm_sub = self.create_subscription(PointStamped, 'arm_position', self.arm_callback, 10)
        self.ctg_linear_sub  = self.create_subscription(Vector3,    'ctg_linear',  self.ctg_linear_callback, 10)
        self.ctg_angular_sub = self.create_subscription(Quaternion, 'ctg_angular', self.ctg_angular_callback, 10)
        if self.state==Rstates.INITIAL:
            # return
            self.state=Rstates.SEARCHING
            # self.get_logger().info('Searching an Aruco')
        if self.state==Rstates.SEARCHING:
            self.arm_pub.publish(Vector3(x=float(0.0),z=float(-0.1)))
            # self.get_logger().info('Start spotting')
            
            self.searcher = self.create_timer(1/60, self.search_aruco)
            
        elif self.state==Rstates.ALIGNING:
            # if self.on_the_ground==False:
            #     self.timer = self.create_timer(1/60, self.timer_callback)
            # else:
            #     self.searcher = self.create_timer(1/60, self.rotate_of_given_theta)
            self.timer = self.create_timer(1/60, self.timer_callback)
        elif self.state==Rstates.MOVING_FORWARD:
            self.destroy_all_timers()
            self.t = time.time()
            self.timer_fw = self.create_timer(1/60, self.move_forward)
        elif self.state==Rstates.GRAB:
            # self.grabber = self.create_timer(1/60, self.grab)
            self.destroy_all_timers()
            self.grab()
            self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0.0)),
            angular=Vector3(z=float(0.0))))
            self.arm_pub.publish(Vector3(x=float(0.0),z=float(0.03)))
        #     self.state=Rstates.PUTDOWN
        #     self.get_logger().info('GRABBED')
        #     # self.start()
            
        # if self.state==Rstates.PUTDOWN:
           
            # while i<3:
            #     self.arm_pub.publish(Vector3(x=float(0.0),z=float(-0.1)))
            #     i+=1
            # self.arm_pub.publish(Vector3(x=float(0.0),z=float(-0.1)))
            self.get_logger().info('PUT DOWN')
            self.on_the_ground=True
            self.state=Rstates.SEARCHING
            self.targetaruco=7
            self.start()
        else:
            self.destroy_all_timers()
            self.get_logger().info('DONE THIS PART')
            # self.done.set_result(1)
            pass

    def grab(self):
        # self.arm_pub.publish(Vector3(x=float(0.3),z=float(0.0)))

        # self.get_logger().info('GRAB GRAB GRAB')
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0.0)),
            angular=Vector3(z=float(0.0))))
        self.t = time.time()
        self.close_gripper()
        # self.arm_pub.publish(Vector3(x=float(0.0),z=float(0.5)))
        self.state=Rstates.PUTDOWN
        # self.destroy_timer(self.grabber)
        # self.start()

    def close_gripper(self):
        # self.get_logger().info("Closing gripper")
        # self.get_logger().info('in method')
        result=None
        # while result is None or (result is not None and not result.done()):
            # self.get_logger().info('waiting')
        future=self.gap.send_goal_async(GripperControl.Goal(target_state=GripperState.CLOSE))
        # try: rclpy.spin_until_future_complete(self, future,timeout_sec=0.5)
        # except KeyboardInterrupt: pass
        # if (result is not None and result.done()):
        #     self.get_logger().info('GRABBEDDDDDD!!!!')
        # else:
        #     self.get_logger().info('NOT GRABBEDDDDDD!!!!')
        # self.state=Rstates.DONE



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
        # self.get_logger().info("I'm moving forward")
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.02), y=float(0)),
            angular=Vector3(z=float(0))))
        # if self.t+0.1 <= time.time():
        if self.t+0.15 <= time.time():
            self.get_logger().info("I am done moving forward")
            self.destroy_timer(self.move_forward)
            self.state=Rstates.GRAB
            self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0)),
            angular=Vector3(z=float(0))))
            self.start()
        elif self.seen is not None and time.time()>self.seen + 0.1:
            self.vel_pub.publish(Twist(
            linear=Vector3(x=float(0.0), y=float(0)),
            angular=Vector3(z=float(0))))
            self.get_logger().info("I am done moving forward")
            # self.destroy_timer(self.move_forward)
            if self.on_the_ground==False:
                self.state=Rstates.GRAB
            else:
                self.state=Rstates.DONE
            self.start()

    # def rotate_of_given_theta(self):
    #     self.get_logger().info("Rotating to align")
    #     aruco_centre=self.getArUcoCentre(self.corners)
    #     error = np.arctan2(np.sin(aruco_centre[0]-self.width/2.0), np.cos(aruco_centre[0]-self.width/2.0)) 
        
    #     error=error%1.57
    #     sign=self.decideTurning(self.corners)
    #     if sign == 'stop':
    #         self.move(0.0,0.0,0.0)
    #         self.aligned = True
    #         self.destroy_all_timers()
    #         sign=None
    #         self.state=Rstates.MOVING_FORWARD
    #         self.get_logger().info('aligned')
    #         self.start()
    #         return
    #     elif self.sign == 'left':
    #         self.aligned = False
    #         self.get_logger().info('left')
    #         sign = -1
    #     else:
    #         self.aligned = False
    #         self.get_logger().info('right')
    #         sign =1

    #     self.velocity=(self.pid.step(error)/1.0)
    #     z = sign * self.velocity
    #     self.get_logger().info(f'moving of: {z}')
    #     self.move(0.0,0.0,z)
        
    def decideTurning(self,corners):
        aruco_centre_x=self.getArUcoCentre(self, corners)[0]
        if np.isclose(aruco_centre_x,self.width/2, 0.01):
            return 'stop'
        elif aruco_centre_x>self.width/2:
            return 'left'
        return 'right'

    def getArUcoCentre(self, corners):
        # get the centre x value
        topL, topR, bottomR, bottomL = corners[0]
        # topL, topR, bottomR, bottomL=tuple(topL), tuple(topR), tuple(bottomR), tuple(bottomL)
        centre_x=(((topL[0]+ topR[0])/2.0)+(bottomR[0]+ bottomL[0])/2.0)/2.0
        centre_y=(((topL[1]+ bottomL[1])/2.0)+(bottomR[1]+ topR[1])/2.0)/2.0
        return [int(centre_x),int(centre_y)]

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
            self.move(0.0,0.0,0.0,0.0,0.0)
            # alpha = np.interp(t, [self.seen, self.seen+3], [1, 0])
            # beta = 1-alpha
            # self.move(
            #     x=alpha*x + beta*-0.2,
            #     y=alpha*y,
            #     z=alpha*z + beta*-0.1,
            #     tz=alpha*tz
            # )

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
        self.height, self.width = msg.height, msg.width
        corners, ids, rejected_img_points = self.aruco.detect(img)
        self.corners=corners

        if ids is not None and self.targetaruco in ids:
            self.state=Rstates.ALIGNING
            self.seen = time.time()
            rvecs, tvecs, objp = self.aruco.get_aruco_poses(corners)
            img = self.aruco.draw_markers(img, corners, ids, rvecs, tvecs)

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            P = self.mktransform(np.matrix(tvec), np.matrix(rvec))
            T = self.mktransform(np.matrix([0., 0., 0.2]), np.matrix([0., 0., 0.]))
            M = P @ T
            # self.get_logger().info(f"{self.ctg_linear}, {self.ctg_angular}")
            if self.ctg_linear is not None and self.ctg_angular is not None:
                ctg = self.mktransform(self.ctg_linear, self.ctg_angular)
                M = M @ ctg
            R = M[:3,:3]
            nrvec, _ = cv2.Rodrigues(R)
            ntvec = M[:3,3]

            theta = nrvec[2]
            self.v = (ntvec, theta)
            speed = (np.abs(ntvec).sum() + abs(theta)) / 4
            if speed < 0.08:
                self.state=Rstates.MOVING_FORWARD
                self.get_logger().info(f'Pass to move forward')
                # self.destroy_timer(self.timer)
                self.start()
            # else: self.get_logger().info(f'speed: {speed}')
            
        self.image_pub.publish(msg)

    def ctg_linear_callback(self, msg: Vector3):
        self.ctg_linear = np.matrix([0., msg.x, msg.z])

    def ctg_angular_callback(self, msg: Quaternion):
        qx, qy, qz, qw = msg.x, msg.y, msg.z, msg.w
        x, y, z = euler_from_quaternion(qx, qy, qz, qw)
        self.ctg_angular = np.matrix([y, 0., 0.])

    def stop(self): self.move()

    def destroy_all_timers(self):
        for timer in self.timers:
            self.destroy_timer(timer)

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