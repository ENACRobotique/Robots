import math
from collections import namedtuple


class Locomotion:
    def __init__(self, robot):
        self.robot = robot
        self.x = None
        self.y = None
        self.theta = None
        self.is_trajectory_finished = True
        self.is_stopped = None
        self.stop_robot()
        self.current_trajectory = []

    def follow_trajectory(self, points, theta, speed):
        if len(points) > 10:
            raise AttributeError("Too much points given -- Message Size restriction")

        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.TRAJECTORY
        message.payload = self.robot.communication.sTrajectory()
        message.payload.nb_traj = len(points)
        message.payload.speed = speed
        message.payload.theta_final = theta
        message.payload.element = []
        for pt in points:
            traj_elt = self.robot.communication.sTrajElement()
            traj_elt.x = pt.x
            traj_elt.y = pt.y
            message.payload.element.append(traj_elt)
        self.robot.communication.send_message(message)

        for i, pt in enumerate(points):
            self.current_trajectory.append(trajectory_point(self.robot.communication._current_msg_id - 1, i, pt))



    def go_to_orient(self, x, y, theta, speed):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.TRAJECTORY
        message.payload = self.robot.communication.sTrajectory()
        message.payload.nb_traj = 1
        message.payload.speed = speed
        message.payload.theta_final = theta
        traj_elt = self.robot.communication.sTrajElement()
        traj_elt.x = x
        traj_elt.y = y
        message.payload.element = [traj_elt]
        self.robot.communication.send_message(message)
        self.current_trajectory.append(trajectory_point(self.robot.communication._current_msg_id - 1, 0, self.Point(x , y)))


    def go_to_orient(self, point, speed):
        self.go_to_orient(point.x, point.y, point.theta, speed)

    def stop_robot(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.STOP
        self.robot.communication.send_message(message)
        self.is_stopped = True

    def restart_robot(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.RESTART
        self.robot.communication.send_message(message)
        self.is_stopped = self.is_trajectory_finished

    def reposition_robot(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.synchronize_position()

    def synchronize_position(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.REPOSITIONING
        message.payload = self.robot.communication.sRepositionning()
        message.payload.x = self.x
        message.payload.y = self.y
        message.payload.theta = self.theta
        self.robot.communication.send_message(message)

    def point_reached(self, traj_id, point_id, x, y, theta):
        index_to_remove = None
        for i, traj_elt in enumerate(self.current_trajectory):
            if traj_elt.traj_id == traj_id and traj_elt.point_number == point_id:
                index_to_remove = i
        if index_to_remove is not None:
            for i in range(index_to_remove + 1):
                self.current_trajectory.pop(0)
        else:
            raise IndexError("Reached Point is not stored !")
        if len(self.current_trajectory) == 0:
            self.is_trajectory_finished = True
            self.is_stopped = True

    def distance_to(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    class PointOrient(Point):
        def __init__(self, x, y, theta=0):
            super().__init__(x, y)
            self.theta = theta


trajectory_point = namedtuple("TrajectoryPoint", ['traj_id','point_number','point'])
