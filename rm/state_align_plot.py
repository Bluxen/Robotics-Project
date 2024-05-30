
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from .state_align import Align, mktransform
import rclpy
import numpy as np

import matplotlib.pyplot as plt

def draw3df(f, ax, name = None):
    """ 
    From https://github.com/alessandro-giusti/teaching-notebooks/blob/master/robotics/02%20transforms3d.ipynb
    
    Draw 3d frame defined by f on axis ax (if provided) or on a new axis otherwise 
    """
    xhat = np.array(f @ np.array([[0,0,0,1],[1,0,0,1]]).T)
    yhat = np.array(f @ np.array([[0,0,0,1],[0,1,0,1]]).T)
    zhat = np.array(f @ np.array([[0,0,0,1],[0,0,1,1]]).T)
    ax.plot(xhat[0,:],xhat[1,:],xhat[2,:],'r-') # transformed x unit vector
    ax.plot(yhat[0,:],yhat[1,:],yhat[2,:],'g-') # transformed y unit vector
    ax.plot(zhat[0,:],zhat[1,:],zhat[2,:],'b-') # transformed z unit vector
    if(name):
        ax.text(xhat[0,0],xhat[1,0],xhat[2,0],name,va="top",ha="center")

# Functions from https://stackoverflow.com/questions/13685386/how-to-set-the-equal-aspect-ratio-for-all-axes-x-y-z
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


class AlignPlot(Align):

    def init(self):
        super().init()
        self.declare_parameter('plot_alignment', rclpy.Parameter.Type.BOOL)
        self.do_plot_alignment = self.get_parameter_or('plot_alignment', None).get_parameter_value().bool_value
        
        if self.do_plot_alignment:
            self.linear_data = []
            self.angular_data = []
            self.linear_twist_data = []
            self.angular_twist_data = []
            self.ts = []
            self.create_subscription(Vector3, 'gripper_linear', self.linear_callback, 10)
            self.create_subscription(Vector3, 'gripper_angular', self.angular_callback, 10)
            self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
            
    def odom_callback(self, msg: Odometry):
        self.linear_twist_data.append(msg.twist.twist.linear)
        self.angular_twist_data.append(msg.twist.twist.angular.z)
        self.ts.append((msg.header.stamp.nanosec / 1e9) + msg.header.stamp.sec)

    def linear_callback(self, msg: Vector3):
        self.linear_data.append(msg)

    def angular_callback(self, msg: Vector3):
        self.angular_data.append(msg)

    def switch_state(self, state):
        if self.do_plot_alignment:
            self.move()
            # Plot velocity over time

            mint = np.min(self.ts)
            self.ts = [t - mint for t in self.ts]
            xs, ys = zip(*[(v.x, v.y) for v in self.linear_twist_data])
            plt.plot(self.ts, xs)
            plt.plot(self.ts, ys)
            plt.plot(self.ts, self.angular_twist_data)
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity')
            plt.legend(['Velocity on the x axis (m/s)', 'Velocity on the y axis (m/s)', 'Angular velocity (rad/s)'])
            plt.show()

            # Plot gripper pose over time
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            xs, ys, zs = zip(*[(p.x * 10, p.y * 10, p.z * 10) for p in self.linear_data])
            ax.scatter(xs, ys, zs)

            for p, t in list(zip(self.linear_data, self.angular_data))[::60]:
                tvec = np.matrix((p.x * 10, p.y * 10, p.z * 10))
                rvec = np.matrix((t.x, t.y, t.z))
                f = mktransform(tvec, rvec)
                draw3df(f, ax)

            ax.set_box_aspect([1,1,1])
            set_axes_equal(ax)
            plt.show()
        return super().switch_state(state)