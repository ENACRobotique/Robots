class Locomotion:
    def __init__(self, robot):
        self.robot = robot
        self.x = None
        self.y = None
        self.theta = None

    def follow_trajectory(self, points, speed):
        pass

    def go_to_orient(self, x, y, theta, point, speed):
        pass

    def go_to_orient(self, point, speed):
        self.go_to_orient(point.x, point.y, point.theta, speed)

    def stop_robot(self):
        pass

    def restart_robot(self):
        pass

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    class PointOrient(Point):
        def __init__(self, x, y, theta=0):
            super().__init__(x, y)
            self.theta = theta



