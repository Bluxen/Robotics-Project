import rclpy
from .state import State
from nav_msgs.msg import Odometry
from math import sqrt
from rclpy.action.client import ActionClient
from robomaster_msgs.msg import GripperState
from robomaster_msgs.action import GripperControl
import time

from .state_follow_thymio import FollowThymio

def distance(pos1, pos2):
    return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2))

class MoveForward(State):

    def init(self):
        self.timer = self.create_timer(1/60, self.move_forward)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.initial_position = None
        self.gap = ActionClient(self, GripperControl, 'gripper')
        self.gap.wait_for_server()

    def move_forward(self):
        self.move(x=0.1)

    def odom_callback(self, odom: Odometry):
        position = odom.pose.pose.position
        if self.initial_position is None:
            self.initial_position = position
        elif distance(self.initial_position, position) >= 0.14:
            self.grab()
            self.destroy_subscription(self.odom_sub)
            self.destroy_timer(self.timer)
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.withdraw, 10)
            # self.gap.destroy()
            # self.switch_state(FollowThymio(74))

    def grab(self):
        future=self.gap.send_goal_async(GripperControl.Goal(target_state=GripperState.CLOSE))
        time.sleep(1.)
        # try: rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        # except KeyboardInterrupt: pass


    def withdraw(self, odom: Odometry):
        position = odom.pose.pose.position
        if self.initial_position is None:
            self.initial_position = position
        self.move(x=-0.1)
        if distance(self.initial_position, position) >= 0.3:
            self.grab()
            self.gap.destroy()
            self.switch_state(FollowThymio())