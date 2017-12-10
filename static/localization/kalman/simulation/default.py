#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <simulation> environment

Feel free to edit this template as you like!
"""

from morse.builder import *

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> simulation' can help you to build custom robots.
robot = Morsy()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(0, 0.0, 0.0)
robot.rotate(0.0, 0.0, 0)
robot.scale = (0.2, 0.2, 0.2)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> simulation' can help you with the creation of a custom
# actuator.
motion = MotionVW()
robot.append(motion)


# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType = 'Position')

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> simulation' can help you with the creation of a custom
# sensor.
pose = Pose()
robot.append(pose)

# Odometry
odometry = Odometry()
robot.append(odometry)
odometry.add_interface('socket')


# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html 
# the other available interfaces (like ROS, YARP...)
robot.add_default_interface('socket')


# set 'fastmode' to True to switch to wireframe mode
env = Environment('./simulation/blender/simple_table.blend', fastmode = False)
env.set_camera_location([-5.0, -4.7, 4.8])
env.set_camera_rotation([1.09, 0, -1.34])

