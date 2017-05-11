import robot
import time

robot = robot.Robot()
while 1:
    input("Press enter to close rocket launcher")
    robot.io.lock_rocket_launcher()
    input("Press enter to open rocket launcher")
    robot.io.open_rocket_launcher()
    input("Press enter to close cannon barrier")
    robot.io.close_cannon_barrier()
    input("Press enter to open cannon barrier")
    robot.io.open_cannon_barrier()
