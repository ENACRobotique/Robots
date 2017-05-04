from communication import *
from io import *

behaviors = {
    "FSMMatch": 0,
    "FSMTests": 1
}


class Robot:
    def __init__(self, behavior=behaviors["FSMMatch"]):
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
                robot.communication.x = msg.x
                robot.communication.y = msg.y
                robot.communication.theta = msg.theta
        robot.behavior.loop()