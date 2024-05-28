import time

from .state import State
from nav_msgs.msg import Odometry
from math import sqrt

def distance(pos1, pos2):
    return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2) + pow(pos1.z - pos2.z, 2))

class MoveForward(State):

    def init(self):
        self.t = time.time()
        self.timer = self.create_timer(1/60, self.move_forward)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.initial_position = None

    def move_forward(self):
        self.move(x=0.1)

    def odom_callback(self, odom: Odometry):
        position = odom.pose.pose.position
        if self.initial_position is None:
            self.initial_position = position
        elif distance(self.initial_position, position) >= 0.15:
            self.switch_state(None)