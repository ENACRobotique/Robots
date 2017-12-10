import math
import time
import numpy as np

WHEEL_BASE = 0.1
Q = np.diag([0.1, 0.1, 0.1])  # Process noise cov matrix (model error)


class State():
    def __init__(self, x=0.0, y=0.0, theta=0.0, date=time.time()):
        self.x = x
        self.y = y
        self.theta = theta
        self.time = date

    def distance(self, other):
        """ Compute distance from other state """
        return np.sqrt(pow(self.x - other.x, 2) + pow(self.y - other.y, 2))

    def turn(self, other):
        """ Compute the angle difference with other state"""
        return abs(self.theta - other.theta)

    def __len__(self):
        return 3

    def __repr__(self):
        return "S : x = {}, y = {}, theta(°) = {}, time = {}"\
            .format(self.x, self.y, self.theta * 180 / math.pi, self.time)

    def __add__(self, column_vector):
        return State(self.x + column_vector[0][0], self.y + column_vector[1][0], self.theta + column_vector[2][0], date=self.time)
