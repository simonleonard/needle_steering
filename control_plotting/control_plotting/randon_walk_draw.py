import rclpy
from rclpy.node import Node
import time
import matplotlib.pyplot as plt
import numpy as np


class RandomWalkerDraw(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(0, 255)
        self.ax.set_ylim(0, 255)
        self.rw = self.randomwalk()
        self.x, self.y = next(self.rw)

        plt.show(block=False)
        plt.draw()

        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)

        self.points = self.ax.plot(self.x, self.y, 'o')[0]
        self.tic = time.time()

    def randomwalk(self, dims=(256, 256), n=20, sigma=5, alpha=0.95, seed=1):
        """ A simple random walk with memory """
        r, c = dims
        gen = np.random.RandomState(seed)
        pos = gen.rand(2, n) * ((r,), (c,))
        old_delta = gen.randn(2, n) * sigma

        while True:
            delta = (1. - alpha) * gen.randn(2, n) * sigma + alpha * old_delta
            pos += delta
            for ii in range(n):
                if not (0. <= pos[0, ii] < r):
                    pos[0, ii] = abs(pos[0, ii] % r)
                if not (0. <= pos[1, ii] < c):
                    pos[1, ii] = abs(pos[1, ii] % c)
            old_delta = delta
            yield pos

    def timer_callback(self):
        self.x, self.y = next(self.rw)
        self.points.set_data(self.x, self.y)
        if self.i == 0:
            self.fig.canvas.draw()
        else:
            # restore background
            self.fig.canvas.restore_region(self.background)

            # redraw just the points
            self.ax.draw_artist(self.points)

            # fill in the axes rectangle
            self.fig.canvas.blit(self.ax.bbox)
        self.i = self.i + 1
        new_tic = time.time()
        print(f"{1.0 / (new_tic - self.tic):.3f}")
        self.tic = new_tic


def main(args=None):
    rclpy.init(args=args)

    drawer = RandomWalkerDraw()

    rclpy.spin(drawer)

    drawer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    # run(doblit=True)
