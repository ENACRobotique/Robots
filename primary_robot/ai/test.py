import robot
import time

robot = robot.Robot()
while 1:
    input("test")
    robot.io.set_led_color(robot.io.LedColor.BLACK)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.RED)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.GREEN)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.BLUE)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.YELLOW)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.PURPLE)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.CYAN)
    time.sleep(0.5)
    robot.io.set_led_color(robot.io.LedColor.WHITE)