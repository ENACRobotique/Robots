import sys
import time
try:
    from pymorse import Morse

except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)
from enum import Enum
from command import Command
from measurement import SimpleMeasurement
from sensor import BasicSensor
from state import State
import numpy as np
from random import normalvariate

BLENDER_X_RANGE = 3
BLENDER_Y_RANGE = 2
REAL_X_RANGE = 3
REAL_Y_RANGE = 2

MORSE_MEASUREMENT_X_STD = MORSE_MEASUREMENT_Y_STD = 0.05  # in m
MORSE_MEASUREMENT_YAW_STD = 3 * np.pi / 180  # in rad

MORSE_ODO_TOTAL_SPEED_STD = 0.3  # in m/s
MORSE_ODO_ROT_SPEED_STD = 15 * np.pi / 180  # in rad/sec



class DATA_TYPE(Enum):
    COMMAND = 0
    MEASUREMENT = 1
    ACTUAL = 2


def get_stimulated_odometry(queue, lock):
    for i in range(1, 50):
        command = Command(10, 0, i/10)
        lock.acquire()
        queue.put((DATA_TYPE.COMMAND, command))
        lock.release()


# Morse part
def blender2real(x, y):
    """ Shift origin to bottom left corner """
    return x + BLENDER_X_RANGE/2, y + BLENDER_Y_RANGE/2


def get_morse_odometry(queue, lock):
    with Morse() as simu:
        while True:

            # Get odometry data on morse
            odometry_data = simu.robot.odometry.get_local_data().result()
            vx = odometry_data['vx']
            vy = odometry_data['vy']
            total_speed = pow(pow(vx, 2) + pow(vy, 2), 0.5)
            rotation_speed = odometry_data['wz']
            current_time = simu.time()

            # Create command
            command = Command(total_speed, rotation_speed, current_time)

            # Add noise if moving
            if command.has_noise():
                command.total_speed += normalvariate(0, MORSE_ODO_TOTAL_SPEED_STD)
                command.rotation_speed += normalvariate(0, MORSE_ODO_ROT_SPEED_STD)

            # Write in queue to process
            lock.acquire()
            queue.put((DATA_TYPE.COMMAND, command))
            lock.release()

            time.sleep(0.05)


def get_morse_measurement(queue, lock):
    morse_sensor = BasicSensor(np.diag([MORSE_MEASUREMENT_X_STD, MORSE_MEASUREMENT_Y_STD, MORSE_MEASUREMENT_YAW_STD]))
    with Morse() as simu:
        while True:
            # Get actual pose
            pose = simu.robot.pose.get_local_data().result()
            x_blender = pose['x']
            y_blender = pose['y']
            yaw = pose['yaw']

            # Shift into table coordinates
            (x, y) = blender2real(x_blender, y_blender)

            # Saved true state
            true_state = State(x, y, yaw)

            # Write actual state in queue to process
            lock.acquire()
            queue.put((DATA_TYPE.ACTUAL, true_state))
            lock.release()

            # Add Gaussian noise
            x_measured = normalvariate(x, MORSE_MEASUREMENT_X_STD)
            y_measured = normalvariate(y, MORSE_MEASUREMENT_Y_STD)
            yaw_measured = normalvariate(yaw, MORSE_MEASUREMENT_YAW_STD) #% (2 * np.pi)
            measurement = SimpleMeasurement(x_measured, y_measured, yaw_measured, morse_sensor)

            # Write measurement in queue to process
            lock.acquire()
            queue.put((DATA_TYPE.MEASUREMENT, measurement))
            lock.release()

            time.sleep(0.2)

