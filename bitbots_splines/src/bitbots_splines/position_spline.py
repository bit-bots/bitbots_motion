from bitbots_splines.polynom import Polynom, pos, vel, acc, jerk
from bitbots_splines.smooth_spline import SmoothSpline, SplinePart, Point


class position_spline():
    def __init__(self, dimensions=6):
        self.splines = []
        for _ in range(dimensions):
            self.splines.append(SmoothSpline())

    def add_point(self, dimension, time, position, velocity=0, acceleration=0):
        self.splines[dimension].add_point(time, position, velocity=velocity, acceleration=acceleration)

    def get_points(self):
        points = []
        for spline in self.splines:
            points.append(spline.get_points())
        return points

    def compute_splines(self):
        for spline in self.splines:
            spline.compute_spline()

    def get_pos(self, t):
        pos = []
        for spline in self.splines:
            pos.append(spline.pos(t))
        return pos

    def get_vel(self, t):
        vel = []
        for spline in self.splines:
            vel.append(spline.vel(t))
        return vel

    def get_acc(self, t):
        acc = []
        for spline in self.splines:
            acc.append(spline.acc(t))
        return acc

    def get_jerk(self, t):
        jerk = []
        for spline in self.splines:
            jerk.append(spline.jerk(t))
        return jerk