#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <particle_filter> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
import math


# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> particle_filter' can help you to build custom robots.
robot = Morsy()

# The list of the main methods to manipulate your components
# is here: http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(0, 0.0, 0.0)
robot.rotate(0.0, 0.0, 3.5)
robot.scale = (0.2, 0.2, 0.2)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> particle_filter' can help you with the creation of a custom
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
# 'morse add sensor <name> particle_filter' can help you with the creation of a custom
# sensor.
pose = Pose()
robot.append(pose)


# Odometry
odometry = Odometry()
robot.append(odometry)
odometry.add_interface('socket')
# odometry.add_stream('get_local_data')

# Camera used as rgb sensor?
# videocamera = VideoCamera()
# videocamera.properties(cam_width=2, cam_height=2)
# videocamera.translate(0.5, 0, 0)
# videocamera.rotate(0, -math.pi/2, 0)
# videocamera.add_interface('socket')
# videocamera.add_stream('socket', port=60005)
# robot.append(videocamera)


# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html
# the other available interfaces (like ROS, YARP...)
robot.add_default_interface('socket')



# set 'fastmode' to True to switch to wireframe mode
env = Environment('../data/simple_table.blend', fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])


