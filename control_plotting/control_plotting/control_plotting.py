import rclpy
from rclpy.node import Node

from needle_steering_control_demo_msgs.msg import ControlDemoPoint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import sys


class ControlPlotting(Node):

    def __init__(self):
        super().__init__('control_plotting')
        self.subscription = self.create_subscription(
            ControlDemoPoint,
            '/demo_point',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.tx = []
        self.ls = []
        self.tz = []
        self.tip_x = []
        self.tip_y = []
        self.tip_z = []

        self.fig, ((self.ax_tx, self.ax_x), (self.ax_tz, self.ax_z)) = plt.subplots(2, 2)

        self.ax_tx.set_xlim([-5, 70])
        self.ax_tx.set_ylim([-5, 5])

        self.ax_tz.set_xlim([-5, 70])
        self.ax_tz.set_ylim([-5, 5])

        self.ls_tx, = self.ax_tx.plot(self.ls, self.tx)
        self.ls_tz, = self.ax_tz.plot(self.ls, self.tz)

        self.ax_x.set_xlim([-5, 70])
        self.ax_x.set_ylim([-5, 5])

        self.ax_z.set_xlim([-5, 70])
        self.ax_z.set_ylim([-5, 5])

        self.tip_y_x, = self.ax_x.plot(self.tip_y, self.tip_x)
        self.tip_y_z, = self.ax_z.plot(self.tip_y, self.tip_z)

    def listener_callback(self, msg):
        print("listener_callback")
        # self.get_logger().info(f'Inputs: {msg.inputs.tx} {msg.inputs.ls} {msg.inputs.tz}, Outputs {msg.outputs.x} {msg.outputs.y} {msg.outputs.z}')
        self.tx.append(msg.inputs.tx)
        self.ls.append(msg.inputs.ls)
        self.tz.append(msg.inputs.tz)
        self.tip_x.append(msg.outputs.x)
        self.tip_y.append(msg.outputs.y)
        self.tip_z.append(msg.outputs.z)

    def update_plot(self, frame):
        print("update_plot")

        rclpy.spin_once(self)

        self.ls_tx.set_data(self.ls, self.tx)
        self.ls_tz.set_data(self.ls, self.tz)

        self.tip_y_x.set_data(self.tip_y, self.tip_x)
        self.tip_y_z.set_data(self.tip_y, self.tip_z)

        return [self.ls_tx, self.ls_tz, self.tip_y_x, self.tip_y_z]


def main(args=None):
    rclpy.init(args=args)

    plot = ControlPlotting()

    ani = FuncAnimation(plot.fig, plot.update_plot)

    plt.show(block=True)
    # rclpy.spin(plot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
