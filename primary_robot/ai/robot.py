import sys

import datetime
import argparse
import builtins

from communication import *
from io_robot import *
from locomotion import *

TRACE_FILE = "/home/pi/ai/log/log_"+str(datetime.datetime.now()).replace(' ', '_')

behaviors = {
    "FSMMatch": 0,
    "FSMTests": 1
}


class Robot(object):
    def __init__(self, behavior=behaviors["FSMMatch"]):
        self.io = IO(self)
        self.communication = Communication()
        self.io.InitializeIOs(self)
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
    # Arguments parsing
    robot.communication.mock_communication = parsed_args.no_teensy
    while True:
        msg = robot.communication.check_message()
        if msg is not None:
            if msg.type == eTypeUp.POSITION:
                print("Position : ", msg.x, msg.y, msg.theta)  
                robot.locomotion.x = msg.x
                robot.locomotion.y = msg.y
                robot.locomotion.theta = msg.theta
            if msg.type == eTypeUp.POINT_REACHED:
                robot.locomotion.point_reached(msg.down_id, msg.point_id, msg.x, msg.y, msg.theta)
            if msg.type == eTypeUp.RECALAGE_OK:
                robot.locomotion.recalage_ok()
        robot.behavior.loop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser("AI option parser")
    parser.add_argument('--no_teensy', action='store_true', default=False,
                        help="Mock communications with teensy")
    parsed_args = parser.parse_args()
    if __debug__:
        print("Debug mode enable")
        #with open(TRACE_FILE, 'w') as sys.stdout:
            #main()
        main()
    else:
        main()


