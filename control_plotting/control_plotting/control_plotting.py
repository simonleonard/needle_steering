import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
from control_reproduce_interfaces.msg import Measurement
from std_srvs.srv import Empty


class Points:
    def __init__(self) -> None:
        self.raw = []
        self.filtered = []

    def add_raw(self, val):
        self.raw.append(val)

    def clear(self):
        self.raw.clear()
        self.filtered.clear()


class Trajectory:
    def __init__(self) -> None:
        self.tx = Points()
        self.ls = Points()
        self.tz = Points()
        self.tip_x = Points()
        self.tip_y = Points()
        self.tip_z = Points()

    def clear(self):
        self.tx.clear()
        self.ls.clear()
        self.tz.clear()
        self.tip_x.clear()
        self.tip_y.clear()
        self.tip_z.clear()


class ControlPlotting(Node):

    def __init__(self):
        super().__init__('control_plotting')

        self.demo_traj = Trajectory()

        self.plot_init()

        self.count = 0

        self.demo_wpt_sub = self.create_subscription(
            Measurement,
            '/demo_wpt',
            self.demo_wpt_callback,
            10)
        self.demo_wpt_sub

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.create_service(Empty, 'clear_demo_way_points',
                            self.clear_demo_way_points_callback)

    def init_draw(self):
        self.demo_ls_tx = self.ax_ls_tx.plot(
            self.demo_traj.ls.raw, self.demo_traj.tx.raw)[0]
        self.demo_ls_tz = self.ax_ls_tz.plot(
            self.demo_traj.ls.raw, self.demo_traj.tz.raw)[0]

        self.demo_y_x = self.ax_y_x.plot(
            self.demo_traj.tip_y.raw, self.demo_traj.tip_x.raw)[0]
        self.demo_y_z = self.ax_y_z.plot(
            self.demo_traj.tip_y.raw, self.demo_traj.tip_y.raw)[0]
        plt.draw()

    def plot_init(self):
        self.fig, ((self.ax_ls_tx, self.ax_y_x),
                   (self.ax_ls_tz, self.ax_y_z)) = plt.subplots(2, 2)
        self.fig.set_figwidth(18)
        self.fig.set_figheight(10)

        for ax in [self.ax_ls_tx, self.ax_ls_tz]:
            ax.set_xlim([-5, 70])
            ax.set_ylim([-5, 5])
            ax.set_xticks(np.linspace(-5, 70, 16))
            ax.set_yticks(np.linspace(-5, 5, 11))
            ax.grid()

        for ax in [self.ax_y_x, self.ax_y_z]:
            ax.set_xlim([-5, 70])
            ax.set_ylim([-10, 10])
            ax.set_xticks(np.linspace(-5, 70, 16))
            ax.set_yticks(np.linspace(-10, 10, 21))
            ax.grid()

        plt.show(block=False)
        self.init_draw()

        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.draw()

    def demo_wpt_callback(self, msg):
        self.demo_traj.tx.add_raw(msg.js.tx)
        self.demo_traj.ls.add_raw(msg.js.ls)
        self.demo_traj.tz.add_raw(msg.js.tz)
        self.demo_traj.tip_x.add_raw(msg.tp.x)
        self.demo_traj.tip_y.add_raw(msg.tp.y)
        self.demo_traj.tip_z.add_raw(msg.tp.z)

    def clear_demo_way_points_callback(self, request, response):
        print("clear_demo_way_points_callback")
        self.demo_traj.clear()

        self.ax_ls_tx.lines.remove(self.demo_ls_tx)
        self.ax_ls_tz.lines.remove(self.demo_ls_tz)
        self.ax_y_x.lines.remove(self.demo_y_x)
        self.ax_y_z.lines.remove(self.demo_y_z)

        self.init_draw()

        return response

    def timer_callback(self):
        self.demo_ls_tx.set_data(self.demo_traj.ls.raw, self.demo_traj.tx.raw)
        self.demo_ls_tz.set_data(self.demo_traj.ls.raw, self.demo_traj.tz.raw)

        self.demo_y_x.set_data(self.demo_traj.tip_y.raw,
                               self.demo_traj.tip_x.raw)
        self.demo_y_z.set_data(self.demo_traj.tip_y.raw,
                               self.demo_traj.tip_z.raw)

        # restore background
        self.fig.canvas.restore_region(self.background)

        # redraw just the points
        self.ax_ls_tx.draw_artist(self.demo_ls_tx)
        self.ax_ls_tz.draw_artist(self.demo_ls_tz)
        self.ax_y_x.draw_artist(self.demo_y_x)
        self.ax_y_z.draw_artist(self.demo_y_z)

        # fill in the axes rectangle
        self.fig.canvas.blit(self.fig.bbox)


def main(args=None):
    rclpy.init(args=args)

    plot = ControlPlotting()
    rclpy.spin(plot)

    plot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
