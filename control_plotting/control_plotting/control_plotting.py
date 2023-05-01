import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
from control_reproduce_interfaces.msg import Measurement
from control_reproduce_interfaces.srv import AddFilteredDemoWpts
from std_srvs.srv import Empty
from .trajectory.trajectory import Trajectory


class ControlPlotting(Node):

    def __init__(self):
        super().__init__('control_plotting')

        self.figure_setup()

        self.trajs = {'demo_js': Trajectory(self.axes['ls_tx'], self.axes['ls_tz'], 'b'),
                      'demo_tp': Trajectory(self.axes['y_x'], self.axes['y_z'],  'b'),
                      'demo_tp_filtered': Trajectory(self.axes['y_x'], self.axes['y_z'],  'g')}

        self.init_drawing()

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

        self.create_service(AddFilteredDemoWpts, 'add_filtered_demo_way_points',
                            self.add_filtered_demo_way_points_callback)

    def figure_setup(self):
        self.fig, ((ls_tx, y_x),
                   (ls_tz, y_z)) = plt.subplots(2, 2)
        self.fig.set_figwidth(18)
        self.fig.set_figheight(10)

        for ax in [ls_tx, ls_tz]:
            ax.set_xlim([-5, 70])
            ax.set_ylim([-5, 5])
            ax.set_xticks(np.linspace(-5, 70, 16))
            ax.set_yticks(np.linspace(-5, 5, 11))
            ax.grid()

        for ax in [y_x, y_z]:
            ax.set_xlim([-5, 70])
            ax.set_ylim([-10, 10])
            ax.set_xticks(np.linspace(-5, 70, 16))
            ax.set_yticks(np.linspace(-10, 10, 21))
            ax.grid()

        self.axes = {'ls_tx': ls_tx, 'ls_tz': ls_tz, 'y_x': y_x, 'y_z': y_z}

    def init_drawing(self):
        plt.show(block=False)

        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.draw()

    def demo_wpt_callback(self, msg):
        self.trajs['demo_js'].add_pt(msg.js.tx, msg.js.ls, msg.js.tz)
        self.trajs['demo_tp'].add_pt(msg.tp.x, msg.tp.y, msg.tp.z)

    def clear_demo_way_points_callback(self, request, response):
        self.trajs['demo_js'].clear_drawing()
        self.trajs['demo_tp'].clear_drawing()
        self.trajs['demo_tp_filtered'].clear_drawing()
        plt.draw()
        return response

    def add_filtered_demo_way_points_callback(self, request, response):
        self.trajs['demo_tp_filtered'].update_all_pts(
            lambda i: [request.tps[i].x, request.tps[i].y, request.tps[i].z], len(request.tps))
        return response

    def timer_callback(self):
        self.fig.canvas.restore_region(self.background)

        for traj in self.trajs.values():
            traj.redraw_pts()

        self.fig.canvas.blit(self.fig.bbox)


def main(args=None):
    rclpy.init(args=args)

    plot = ControlPlotting()
    rclpy.spin(plot)

    plot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
