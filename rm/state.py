from rclpy.node import Node
from asyncio import Future

from geometry_msgs.msg import Twist, Vector3, PointStamped
from sensor_msgs.msg import Image
from .aruco import Aruco
import rclpy

from abc import abstractmethod

class State(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.vel_pub   = self.create_publisher(Twist,   'cmd_vel',   10)
        self.arm_pub   = self.create_publisher(Vector3, 'cmd_arm',   10)
        self.arm_sub = self.create_subscription(PointStamped, 'arm_position', self.arm_callback, 10)
        self.image_pub = self.create_publisher(Image,   'debug_img', 10)
        self.arm_x = 1.
        self.get_logger().info("State initialized")
        self.done = Future()

        # Set calibration type if this is the first state
        if Aruco.optimal_calibration is None:
            self.declare_parameter('use_calibration', rclpy.Parameter.Type.BOOL)
            Aruco.optimal_calibration = not self.get_parameter_or('use_calibration', None).get_parameter_value().bool_value
            self.get_logger().info(f"Using camera type: {'optimal' if Aruco.optimal_calibration else 'calibration'}")

    def switch_state(self, state):
        self.done.set_result(state)

    @abstractmethod
    def init(self):
        self.get_logger().warn("Init method not defined for this state")

    def move(self, x=0, y=0, z=0, tx=0, ty=0, tz=0):
        if self.arm_x < 0.178 and x>0:
            arm_x, body_x = 0.4*x, 0.6*x
        else:
            arm_x, body_x = 0., x
        self.vel_pub.publish(Twist(
            linear=Vector3(x=float(body_x), y=float(y)),
            angular=Vector3(z=float(tz))))
        self.arm_pub.publish(Vector3(x=float(arm_x),z=float(z)))

    def stop(self): 
        self.move()
        self.vel_pub.wait_for_all_acked()

    def arm_callback(self, msg):
        self.arm_x = msg.point.x