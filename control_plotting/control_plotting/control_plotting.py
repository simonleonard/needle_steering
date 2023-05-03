import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import matplotlib.pyplot as plt
import numpy as np

from control_reproduce_interfaces.msg import Measurement, TipPosition
from control_reproduce_interfaces.srv import AddFilteredDemoWpts
from control_reproduce_interfaces.srv import TogglePlot

from .trajectory.trajectory import Trajectory


class ControlPlotting(Node):

    def __init__(self):
        super().__init__('control_plotting')

        self.figure_setup()

        self.trajs = {'demo_js': Trajectory(self.axes['ls_tx'], self.axes['ls_tz'],
                                            label='demo', color='k', linestyle='-', linewidth=1),
                      'demo_tp': Trajectory(self.axes['y_x'], self.axes['y_z'],
                                            label='demo', color='k', linestyle='-', linewidth=0.75),
                      'demo_tp_filtered': Trajectory(self.axes['y_x'], self.axes['y_z'],
                                                     label='demo_filtered', color='b',
                                                     linestyle='-', linewidth=0.75),
                      'repr_js': Trajectory(self.axes['ls_tx'], self.axes['ls_tz'],
                                            label='reprpduce', color='tab:orange', linestyle='-', linewidth=1),
                      'repr_tp': Trajectory(self.axes['y_x'], self.axes['y_z'],
                                            label='reprpduce', color='tab:red', linestyle='-', linewidth=0.75),
                      'repr_tp_filtered': Trajectory(self.axes['y_x'], self.axes['y_z'],
                                                     label='reprpduce_filtered', color='tab:orange',
                                                     linestyle='-', linewidth=0.75),
                      'jacobian_update_tp': Trajectory(self.axes['y_x'], self.axes['y_z'],
                                                       label='jacobian update', color='m',
                                                       linestyle=None, linewidth=0.0,
                                                       marker='*', markersize=0.75, show=False)}

        self.init_drawing()

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscriptions_setup()
        self.services_setup()

    def figure_setup(self):
        self.fig, ((ls_tx, y_x),
                   (ls_tz, y_z)) = plt.subplots(2, 2)
        self.fig.set_figwidth(18)
        self.fig.set_figheight(10)

        ls_tx.set_title('Joint state TX vs.LS')
        ls_tz.set_title('Joint state TZ vs.LS')
        y_x.set_title('Tip Position x vs. y')
        y_z.set_title('Tip Position z vs. y')

        ls_tx.set_xlabel('LS (mm)')
        ls_tz.set_xlabel('LS (mm)')
        y_x.set_xlabel('y (mm)')
        y_z.set_xlabel('y (mm)')
        ls_tx.set_ylabel('TX (mm)')
        ls_tz.set_ylabel('TZ (mm)')
        y_x.set_ylabel('x (mm)')
        y_z.set_ylabel('z (mm)')

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

    def subscriptions_setup(self):
        self.demo_wpt_sub = self.create_subscription(
            Measurement,
            '/demo_wpt',
            self.demo_wpt_callback,
            10)
        self.demo_wpt_sub

        self.repr_wpt_sub = self.create_subscription(
            Measurement,
            '/reproduce_wpt',
            self.repr_wpt_callback,
            10)
        self.repr_wpt_sub

        self.repr_tp_filtered_sub = self.create_subscription(
            TipPosition,
            '/reproduce_tp_filtered',
            self.repr_tp_filtered_callback,
            10)
        self.repr_tp_filtered_sub

        self.repr_tp_filtered_sub = self.create_subscription(
            TipPosition,
            '/jacobian_update_tp',
            self.jacobian_update_tp_callback,
            10)
        self.repr_tp_filtered_sub

    def services_setup(self):
        self.create_service(Empty, 'clear_demo_way_points',
                            self.clear_demo_way_points_callback)

        self.create_service(AddFilteredDemoWpts, 'add_filtered_demo_way_points',
                            self.add_filtered_demo_way_points_callback)

        self.create_service(Empty, 'clear_reproduce_way_points',
                            self.clear_reproduce_way_points_callback)

        self.create_service(TogglePlot, 'toggle_plot',
                            self.toggle_plot_callback)

    def init_drawing(self):
        plt.show(block=False)

        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self.fig.canvas.draw()

    def demo_wpt_callback(self, msg):
        self.trajs['demo_js'].add_pt(msg.js.tx, msg.js.ls, msg.js.tz)
        self.trajs['demo_tp'].add_pt(msg.tp.x, msg.tp.y, msg.tp.z)

    def repr_wpt_callback(self, msg):
        self.trajs['repr_js'].add_pt(msg.js.tx, msg.js.ls, msg.js.tz)
        self.trajs['repr_tp'].add_pt(msg.tp.x, msg.tp.y, msg.tp.z)

    def repr_tp_filtered_callback(self, msg):
        self.trajs['repr_tp_filtered'].add_pt(msg.x, msg.y, msg.z)

    def jacobian_update_tp_callback(self, msg):
        self.trajs['jacobian_update_tp'].add_pt(msg.x, msg.y, msg.z)

    def clear_demo_way_points_callback(self, request, response):
        self.trajs['demo_js'].clear_drawing()
        self.trajs['demo_tp'].clear_drawing()
        self.trajs['demo_tp_filtered'].clear_drawing()
        plt.draw()
        return response

    def clear_reproduce_way_points_callback(self, request, response):
        self.trajs['repr_js'].clear_drawing()
        self.trajs['repr_tp'].clear_drawing()
        self.trajs['repr_tp_filtered'].clear_drawing()
        self.trajs['jacobian_update_tp'].clear_drawing()
        plt.draw()
        return response

    def toggle_plot_callback(self, request, response):
        if request.name == 'demo':
            self.trajs['demo_tp'].toggle_visualization()
            response.status = request.SHOWN if self.trajs['demo_tp'].show else request.HIDDEN
        elif request.name == 'demo filtered':
            self.trajs['demo_tp_filtered'].toggle_visualization()
            response.status = request.SHOWN if self.trajs['demo_tp_filtered'].show else request.HIDDEN
        elif request.name == 'reproduce':
            self.trajs['repr_js'].toggle_visualization()
            self.trajs['repr_tp'].toggle_visualization()
            response.status = request.SHOWN if self.trajs['repr_js'].show else request.HIDDEN
        elif request.name == 'reproduce filtered':
            self.trajs['repr_tp_filtered'].toggle_visualization()
            response.status = request.SHOWN if self.trajs['repr_tp_filtered'].show else request.HIDDEN
        elif request.name == 'jacobian update':
            self.trajs['jacobian_update_tp'].toggle_visualization()
            response.status = request.SHOWN if self.trajs['jacobian_update_tp'].show else request.HIDDEN
        else:
            response.status = request.FAILED
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
