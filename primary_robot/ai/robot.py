import sys

import datetime

from communication import *
from io_robot import *
from locomotion import *

TRACE_FILE = "log/AI_LOG"+str(datetime.datetime.now()).replace(' ', '')

behaviors = {
    "FSMMatch": 0,
    "FSMTests": 1
}


class Robot(object):
    def __init__(self, behavior=behaviors["FSMMatch"]):
        self.communication = Communication()
        self.io = IO(self)
        self.locomotion = Locomotion(self)
        if behavior == behaviors["FSMMatch"]:
            from fsmmatch import FSMMatch
            self.behavior = FSMMatch(self)
        elif behavior == behaviors["FSMTests"]:
            raise NotImplementedError("This behavior is not implemented yet !")
        else:
            raise NotImplementedError("This behavior is not implemented yet !")

def main():
    robot = Robot()
    while True:
        msg = robot.communication.check_message()
        if msg is not None:
            if msg.type == eTypeUp.POSITION:
                robot.locomotion.x = msg.x
                robot.locomotion.y = msg.y
                robot.locomotion.theta = msg.theta
            if msg.type == eTypeUp.POINT_REACHED:
                robot.locomotion.point_reached(msg.down_id, msg.point_id, msg.x, msg.y, msg.theta)
            if msg.type == eTypeUp.RECALAGE_OK:
                robot.locomotion.recalage_ok()
        robot.behavior.loop()


if __name__ == '__main__':
    if __debug__:
        with open(TRACE_FILE, 'w') as sys.stdout:
            main()
    else:
        main()


