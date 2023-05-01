class Trajectory:
    def __init__(self, ax_x, ax_z, style):
        self.ax_x = ax_x
        self.ax_z = ax_z
        self.style = style
        self.xs = []
        self.ys = []
        self.zs = []
        self.__reset_artist()
        # self.show = True

    def add_pt(self, x, y, z):
        self.xs.append(x)
        self.ys.append(y)
        self.zs.append(z)

    def update_all_pts(self, itr, size):
        self.__clear_all_pts()
        for i in range(size):
            self.add_pt(*itr(i))

    def redraw_pts(self):
        self.y_x.set_data(self.ys, self.xs)
        self.y_z.set_data(self.ys, self.zs)
        self.ax_x.draw_artist(self.y_x)
        self.ax_z.draw_artist(self.y_z)

    def clear_drawing(self):
        self.__clear_all_pts()
        self.__clear_drawing()

    # def toggle_visualization(self):
    #     if self.show:
    #         self.show = False
    #     else:
    #         self.show = True

    def __reset_artist(self):
        self.y_x = self.ax_x.plot([], [], self.style)[0]
        self.y_z = self.ax_z.plot([], [], self.style)[0]

    def __clear_drawing(self):
        self.ax_x.lines.remove(self.y_x)
        self.ax_z.lines.remove(self.y_z)
        self.__reset_artist()

    def __clear_all_pts(self):
        self.xs.clear()
        self.ys.clear()
        self.zs.clear()
