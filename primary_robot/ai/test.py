import robot
import time

robot = robot.Robot()
while 1:
    input("test")
    print("Front : " + str(robot.io.front_distance))
    print("Rear : " + str(robot.io.rear_distance))