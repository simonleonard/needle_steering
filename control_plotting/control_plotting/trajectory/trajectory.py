class Trajectory:
    def __init__(self, ax_x, ax_z, label, color, linestyle, linewidth, marker=None, markersize=1, show=True):
        self.ax_x = ax_x
        self.ax_z = ax_z
        self.label = label
        self.color = color
        self.linestyle = linestyle
        self.linewidth = linewidth
        self.marker = marker
        self.markersize = markersize
        self.show = show
        self.xs = []
        self.ys = []
        self.zs = []
        if self.show:
            self.__reset_artist()

    def add_pt(self, x, y, z):
        self.xs.append(x)
        self.ys.append(y)
        self.zs.append(z)

    def update_all_pts(self, itr, size):
        self.__clear_all_pts()
        for i in range(size):
            self.add_pt(*itr(i))

    def redraw_pts(self):
        if self.show:
            self.y_x.set_data(self.ys, self.xs)
            self.y_z.set_data(self.ys, self.zs)
            self.ax_x.draw_artist(self.y_x)
            self.ax_z.draw_artist(self.y_z)

    def clear_drawing(self):
        if self.show:
            self.__clear_all_pts()
            self.__remove_artist()
            self.__reset_artist()

    def toggle_visualization(self):
        if self.show:
            self.__hide_artist()
        else:
            self.__show_artist()

    def __clear_all_pts(self):
        self.xs.clear()
        self.ys.clear()
        self.zs.clear()

    def __reset_artist(self):
        self.y_x = self.ax_x.plot(self.ys, self.xs, label=self.label,
                                  color=self.color, ls=self.linestyle, lw=self.linewidth,
                                  marker=self.marker, ms=self.markersize)[0]
        self.y_z = self.ax_z.plot(self.ys, self.zs, label=self.label,
                                  color=self.color, ls=self.linestyle, lw=self.linewidth,
                                  marker=self.marker, ms=self.markersize)[0]
        self.ax_x.legend(loc="upper left", fontsize="8")
        self.ax_z.legend(loc="upper left", fontsize="8")

    def __remove_artist(self):
        self.ax_x.lines.remove(self.y_x)
        self.ax_z.lines.remove(self.y_z)
        self.ax_x.legend(loc="upper left", fontsize="8")
        self.ax_z.legend(loc="upper left", fontsize="8")

    def __hide_artist(self):
        self.show = False
        self.__remove_artist()

    def __show_artist(self):
        self.show = True
        self.__reset_artist()
