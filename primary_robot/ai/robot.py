import sys

import datetime
import argparse
import os

from communication import *
from io_robot import *
from robot_ivy import Ivy
from locomotion import *
from obstacles import load_obstacles_from_file

#TRACE_FILE = "/home/pi/code/primary_robot/ai/log/log_"+str(datetime.datetime.now()).replace(' ', '_')
TRACE_FILE = "plop.log"
OBSTACLE_FILE = "obstacles2017.yml"

behaviors = {
    "FSMMatch": 0,
    "FSMTests": 1,
    "Slave" : 2
}


class Robot(object):
    def __init__(self, behavior=behaviors["FSMMatch"]):
        self.obstacles = load_obstacles_from_file(OBSTACLE_FILE)
        if parsed_args.ivy_address is not None:
            self.ivy = Ivy(self, parsed_args.ivy_address)
        self.communication = Communication()
        self.io = IO(self)
        self.locomotion = Locomotion(self)
        if behavior == behaviors["FSMMatch"]:
            from fsmmatch import FSMMatch
            self.behavior = FSMMatch(self)
        elif behavior == behaviors["FSMTests"]:
            raise NotImplementedError("This behavior is not implemented yet !")
        elif behavior == behaviors["Slave"]:
            from slave import Slave
            self.behavior = Slave(self)
        else:
            raise NotImplementedError("This behavior is not implemented yet !")


def main():
    robot = Robot(2)
    # Arguments parsing
    robot.communication.mock_communication = parsed_args.no_teensy
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
    parser = argparse.ArgumentParser("AI option parser")
    parser.add_argument('-t', '--no_teensy', action='store_true', default=False,
                        help="Mock communications with teensy")
    parser.add_argument('-p', '--pygargue', action='store', default=None, dest='ivy_address', type=str)
    parsed_args = parser.parse_args()
    if __debug__:
        with open(TRACE_FILE, 'w') as sys.stdout:
            main()
    else:
        main()


