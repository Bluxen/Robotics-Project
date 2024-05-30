import rclpy
from rclpy.node import Parameter, get_default_context
import sys
import os
from .state_align_plot import AlignPlot
from .state_move_forward import MoveForward
from .state_follow_thymio import FollowThymio

def main():
    rclpy.init(args = sys.argv)
    os.environ["XDG_SESSION_TYPE"] = "xcb"
    state = AlignPlot()

    while state is not None:
        node = state
        node.init()

        # Keep processings events until someone manually shuts down the node
        try: rclpy.spin_until_future_complete(node, node.done)
        except KeyboardInterrupt: break

        node.stop()
        node.destroy_node()
        state = node.done.result()

if __name__ == '__main__':
    main()