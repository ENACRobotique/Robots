class Locomotion:
    def __init__(self, robot):
        self.robot = robot
        self.x = None
        self.y = None
        self.theta = None

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

    def go_to_orient(self, point, speed):
        self.go_to_orient(point.x, point.y, point.theta, speed)

    def stop_robot(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.STOP
        self.robot.communication.send_message(message)

    def restart_robot(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.RESTART
        self.robot.communication.send_message(message)

    def synchronize_position(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.REPOSITIONING
        message.payload = self.robot.communication.sRepositionning()
        message.payload.x = self.x
        message.payload.y = self.y
        message.payload.theta = self.theta
        self.robot.communication.send_message(message)

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    class PointOrient(Point):
        def __init__(self, x, y, theta=0):
            super().__init__(x, y)
            self.theta = theta



