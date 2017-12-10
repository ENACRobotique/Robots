import math


class SimpleMeasurement():
    def __init__(self, x, y, theta, sensor):
        self.x = x
        self.y = y
        self.theta = theta
        self.sensor = sensor

    def __repr__(self):
        return "M : x = {}, y = {}, theta(°) = {}".format(self.x, self.y, self.theta * 180 / math.pi)
