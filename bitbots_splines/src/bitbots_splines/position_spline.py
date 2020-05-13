from bitbots_splines.polynom import Polynom, pos, vel, acc, jerk
from bitbots_splines.smooth_spline import SmoothSpline, SplinePart, Point


class position_spline():
    def __init__(self, dimensions=6):
        self.dimensions = dimensions
        self.splines = []
        for _ in range(self.dimensions):
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
