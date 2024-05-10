import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from  sensor_msgs.msg import Range
from sensor_msgs.msg import Image,CameraInfo

# NEW IMPORTS
import PIL
import numpy as np
import cv2
import os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from .arucoHelperclass import arucoHelper
from .visualServoing import VS

from ament_index_python.packages import get_package_share_directory
SHARE = get_package_share_directory('project')+'/'

import sys

class firstController(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.

        ################## CHANGED ########################
        self.cnt=0
        self.infty_toggle = False
        self.min_distance=1.0
        self.tolerance = 0.05
        self.align_counter = 0

        self.state = 'MOVE_FORWARD'
        self.start_pose = None

        # variables to pass from exercise 1 to 2
        # self.time=1600
        self.time=3000
        self.current_theta=None
        self.current_z=None
        self.angle_tolerance=0.01
        # self.angle_tolerance=0.002
        self.inPlace=False

        self.firstpart_inf=True
        self.secondpart_inf=False
        self.sign=''

        self.helper_aruco = arucoHelper(logger = self.get_logger())
        self.theta_target = 0.05
        self.arucoSpotted = False
        self.aligned = False
        self.start_theta=None
        self.aruco_ids_target:int=55

        self.image_publisher = self.create_publisher(Image, "debug_img", 10)
        self.vs = VS(320, 640)
        
    def start(self): 
        self.get_logger().info('I am back in start')
        if not self.arucoSpotted:
            self.get_logger().info('Start spotting')
            self.image_subscription = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
            self.timer = self.create_timer(1/60, self.search_aruco)
            # self.stopper = self.create_timer(10/60, self.stop_to_check)
        elif not self.aligned:
            self.get_logger().info('Time to align')
            self.start_theta=self.current_theta
            self.timer = self.create_timer(1/60, self.rotate_of_given_theta)
            # self.stopper = self.create_timer(10/60, self.stop_to_check)
        else:
            self.get_logger().info("DONE MY JOB, SEE YOU")
        # self.scan_timer_rotate = self.create_timer(1/60, lambda: self.move(0.,0.,0.1))
        # self.scan_timer_calibr = self.create_timer(3/60, lambda: self.move(0.,0.,0. ))
        # self.scan_timer_stop   = self.create_timer()
        
        # self.get_camera_conf=self.create_subscription(CameraInfo, 'camera/camera_info', self.get_config, 10)
        
        # Create and immediately start a timer that will regularly publish commands
        # self.timer = self.create_timer(1/60, self.update_callback)
        ###################### CHANGED  ##############################
        
        # if self.time!=0:
        #     self.timer = self.create_timer(1/60, self.drawinf_callback)
        # if self.time==0 and not self.inPlace:
        # self.timer = self.create_timer(1/60, self.rotate_straight)
        # self.stopper = self.create_timer(2/60, self.stop_to_check)
        # else:
            
        # self.timer = self.create_timer(1/60, self.update_callback)
            # create the subscription to the proximity sensors
            # self.prox_sensor_center = self.create_subscription(Range, 'proximity/center', self.check_prox_c, 10)
            # self.prox_sensor_centerleft = self.create_subscription(Range, 'proximity/center_left', self.check_prox_cleft, 10)
            # self.prox_sensor_centerright = self.create_subscription(Range, 'proximity/center_right', self.check_prox_cright, 10)
            # self.prox_sensor_rearleft = self.create_subscription(Range, 'proximity/rear_left', self.check_prox_rleft, 10)
            # self.prox_sensor_rearright = self.create_subscription(Range, 'proximity/rear_right', self.check_prox_rright, 10)
            
        # self.isStopped=False
        # self.position_centered=False

        #create a subscription to the images
        # print('subscription created')
        # self.image_subscription = self.create_subscription(Image, '/camera/image_color', self.img_callback, 10)
        
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        ###################################
       
        _,_,self.current_theta=pose2d
        self.current_z=self.odom_valocity
        self.current_theta=round(self.current_theta,6)
        
        ###################################
        
        # self.get_logger().info(
        #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        # )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
        
    def update_callback(self):
        print('in callback')
        self.cmd_vel = Twist() 
        ########################################################
        
        # if self.state == 'MOVE_FORWARD':
        #     self.get_logger().info("I'm moving towards the wall", throttle_duration_sec = 1)
        #     self.cmd_vel.linear.x  = 0.2
        #     if (self.dist_centre > -1 and self.dist_centre < self.min_distance):
        #         self.cmd_vel.linear.x  = 0.0
        #         self.state = 'ALIGN'

        # elif self.state == 'ALIGN':
        #     self.get_logger().info("I'm aligning to the wall", throttle_duration_sec = 1)
        #     error = self.dist_cleft - self.dist_cright
        #     if (error > 0.0 and self.dist_cright > 0.0) or self.dist_cleft < 0.0:
        #         self.cmd_vel.angular.z = -0.5
        #     elif (error < 0.0 and self.dist_cleft > 0.0) or self.dist_cright < 0.0:
        #         self.cmd_vel.angular.z = 0.5
        #     self.align_counter += 1
        #     if self.align_counter >= 120: # wait for error tolerance
        #         self.cmd_vel.angular.z = 0.0
        #         self.state = 'TURN_180'
        #         self.align_counter = 0

        # elif self.state == 'TURN_180':
        #     self.get_logger().info("I'm turning away from the wall", throttle_duration_sec = 1)
        #     self.cmd_vel.angular.z = 1.57079632679
        #     self.align_counter += 1
        #     if (self.align_counter >= 120):
        #         self.cmd_vel.angular.z = 0.0
        #         self.state = 'REAR_ALIGN'
        #         self.align_counter = 0

        # elif self.state == 'REAR_ALIGN':
        #     self.get_logger().info("I'm aligning by butt to the wall", throttle_duration_sec = 1)
        #     error = self.dist_rleft - self.dist_rright
        #     if (error > 0.0 and self.dist_rright > 0.0) or self.dist_rleft < 0.0:
        #         self.cmd_vel.angular.z = 0.2
        #     elif (error < 0.0 and self.dist_rleft > 0.0) or self.dist_rright < 0.0:
        #         self.cmd_vel.angular.z = -0.2
        #     self.align_counter += 1
        #     if self.align_counter >= 120: # wait for error tolerance
        #         self.cmd_vel.angular.z = 0.0
        #         self.state = 'MOVE_2M'
        #         self.align_counter = 0

        # elif self.state == 'MOVE_2M':
        #     self.get_logger().info("I'm moving away from the wall", throttle_duration_sec = 1)
        #     if (self.start_pose == None):
        #         self.start_pose = self.pose3d_to_2d(self.odom_pose)
            
        #     self.cmd_vel.linear.x = 0.2
        #     current_pose = self.pose3d_to_2d(self.odom_pose)
        #     start_pose = self.start_pose
        #     distance_moved = ((current_pose[0] - start_pose[0]) ** 2 + (current_pose[1] - start_pose[1]) ** 2) ** 0.5
        #     self.get_logger().info("I'm {:.2f} m from the wall".format(distance_moved), throttle_duration_sec = 1)
        #     if distance_moved >= 2.0:
        #         self.state = 'STOP'
        #         self.start_pose = None

        # elif self.state == 'STOP':
        #     self.get_logger().info("I'm stopped", throttle_duration_sec = 1)
        #     self.cmd_vel.linear.x  = 0.0
        #     self.cmd_vel.angular.z = 0.0
        #     self.align_counter += 1
        #     if self.align_counter >= 200: # wait a bit
        #         self.destroy_timer(self.timer)
        #         self.state = 'FINISHED'
        #         self.align_counter = 0
        #         self.stop()
                

        # elif self.state == 'ROTATE':
        #     self.cmd_vel.angular.z = 0.2
        
        # #########################################################  
        # # Publish the command
        # self.vel_publisher.publish(self.cmd_vel)
        


    ###################################################
    ################## NEW METHODS ####################
    def stop_to_check(self):
        self.move(0.0,0.0,0.0)

    def move(self,x_linear=0.0,y_linear=0.0, z_angular=0.0):
        cmd_vel = Twist() 
        cmd_vel.linear.x  = x_linear # [m/s]
        cmd_vel.angular.y = y_linear# [rad/s]
        cmd_vel.angular.z = z_angular# [rad/s]
        self.vel_publisher.publish(cmd_vel)
        
    # def rotate_straight(self):
    #     if self.current_theta:
            
    #         if abs(self.current_theta)>self.angle_tolerance:
                
    #             # self.cmd_vel.linear.x  = 0.0 # [m/s]
    #             # self.cmd_vel.linear.y  = 0.0 # [m/s]
    #             if self.current_theta > 0:
    #                 sign = -1
    #             else:
    #                 sign = 1

                   
    #             z = sign*0.2
    #             self.move(0.0,0.0,z)
    #             return
        
    #     # remove the timer to 
    #     self.destroy_timer(self.timer)
    #     self.destroy_timer(self.stopper)
    #     self.inPlace = True
    #     self.start()
        

    def rotate_of_given_theta(self):
        while not self.current_theta:
            self.get_logger().info("Waiting a bit", throttle_duration_sec = 1)
            return
        
        self.get_logger().info("Rotating to align")
        
        # if not np.abs(self.current_theta- self.start_theta) > self.theta_target:
        if self.sign == 'stop':
            self.move(0.0,0.0,0.0)
            # now that we are aligned, no aruco is found
            self.move(0.0,0.0,0.0)
            self.aligned=True
            self.get_logger().info('aligned')
            self.destroy_timer(self.timer)
            # self.destroy_timer(self.stopper)
            self.start()
            return

        elif self.sign == 'left':
            self.aligned=False
            self.get_logger().info('left')
            sign = 1
        else:
            self.aligned=False
            self.get_logger().info('right')
            sign =-1

        z = sign* 0.2
        self.get_logger().info(f'moving of: {z}')
        self.move(0.0,0.0,z)

    def img_callback(self,msg):
        uint8_array=np.asarray(msg.data)
        # print(uint8_array.shape)
        height, width=msg.height, msg.width
        uint8_array=uint8_array.reshape((height, width, 3))
        
        # self.get_logger().info(f"{img}")
        corners, ids, rejected_img_points = self.helper_aruco.getArucoPosition(uint8_array)
        # self.get_logger().info(f"{rejected_img_points}")
        # self.get_logger().info(f"{corners}")

        if ids is not None:
            uint8_array = self.helper_aruco.draw_markers(uint8_array, corners, ids)
            uint8_array = self.vs.draw_pose(uint8_array, corners, ids, 0.05)
        # for square in rejected_img_points:
        #     uint8_array = self.helper_aruco.draw_square(uint8_array, square, (255, 0, 0))
        self.publish_image(uint8_array, msg)
        
        if ids is None:
            if self.arucoSpotted:
                # in case the aruco marker is spotted but at this moment we cannot see it because it is covered
                # by the gripper, we keep on going with the values we have
                return
            self.get_logger().info('\nNo aruco markers found\n')
        else:
            self.arucoSpotted = True
            self.get_logger().info(f'\nAruco markers found\n')
            self.get_logger().info("TARGET ID {}".format(self.aruco_ids_target))
            # id=[id[0] for id in ids if id[0]==self.aruco_ids_target]
            # self.get_logger().info("ID {}".format(id))
            # ids_formatted=ids_formatted[0]
            
            try:
                # the index method returns a particular exception if none of the ids is the correct one
                i=list.index(ids.tolist(), [self.aruco_ids_target])
                self.get_logger().info("FOUND AT {}".format(corners[i]))
                c=corners[i]
                self.sign=self.helper_aruco.decideDirection(c)
                self.get_logger().info("it is time to go: {}".format(self.sign))
            except ValueError as v:
                self.arucoSpotted = False
                self.get_logger().info("FOUND FALSE ARUCO")
            # self.helper_aruco.drawImage(uint8_array, corners)
            # self.helper_aruco.drawImage(uint8_array, rejected_img_points)

            # self.get_logger().info(f'{ids}')

        # img=PIL.Image.fromarray(uint8_array).convert('RGB')
        

        # self.get_logger().info(f'TYPE: {type(img)}')
        # img.save(SHARE+"test.jpg","JPEG")
        # self.get_logger().info('SAVED')
        
        #ax.axis('off')
        

    # def get_config(self,msg):
    #     print('getting config')
    #     self.height, self.width=msg.height, msg.width
    #     print(self.height, self.width)
    #     # print('ROI')
    #     # print(msg.roi.height, msg.roi.width)
    #     self.image_subscription = self.create_subscription(Image, 'camera/image_color', self.img_callback, 10)
        
    #     self.destroy_subscription(self.get_camera_conf)
        
    def search_aruco(self):
        while not self.current_theta:
            self.get_logger().info("Waiting a bit")
            return
        
        if self.arucoSpotted:
            self.get_logger().info("ARUCO SPOTTED")
            self.move(0.0, 0.0, 0.0)
            self.destroy_timer(self.timer)
            # self.destroy_timer(self.stopper)
            self.start()
            return
        
        if self.current_theta:
            
            # if not np.isclose(self.theta_target,self.current_theta,self.angle_tolerance):
            if True:
                # self.get_logger().info(f'DISTANCE: {np.linalg.norm(np.array(self.theta_target) - np.array(self.current_theta))}')

                z = 0.1
                
                self.move(0.0, 0.0, z)
                
                return

    ###################################################

    def publish_image(self, mat: np.ndarray, msg: Image):
        arr = np.reshape(mat, -1)
        msg.data = arr.tolist()

        self.image_publisher.publish(msg)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    os.environ["XDG_SESSION_TYPE"] = "xcb"
    
    # Create an instance of your node class
    node = firstController()
    print('starting the controller')
    node.start()

    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()