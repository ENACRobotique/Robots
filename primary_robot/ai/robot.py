from communication import *
from io_robot import *

behaviors = {
    "FSMMatch": 0,
    "FSMTests": 1
}


class Robot(object):
    def __init__(self, behavior=behaviors["FSMMatch"]):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.communication = Communication()
        self.io = IO(self)
        if behavior == behaviors["FSMMatch"]:
            from fsmmatch import FSMMatch
            self.behavior = FSMMatch(self)
        elif behavior == behaviors["FSMTests"]:
            raise NotImplementedError("This behavior is not implemented yet !")
        else:
            raise NotImplementedError("This behavior is not implemented yet !")


if __name__ == '__main__':
    robot = Robot()
    while True:
        msg = robot.communication.check_message()
        if msg is not None:
            if msg.type == eTypeUp.POSITION or msg.type == eTypeUp.POINT_REACHED:
                robot.x = msg.x
                robot.y = msg.y
                robot.theta = msg.theta
        robot.behavior.loop()
