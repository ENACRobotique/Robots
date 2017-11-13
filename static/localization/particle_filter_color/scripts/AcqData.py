import time
import sys
try:
    from pymorse import Morse, Robot

except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

from scipy import misc
TABLE_IMAGE_PATH = "../data/test2.png"
BLENDER_X_RANGE = 3
BLENDER_Y_RANGE = 2
REAL_X_RANGE = 3
REAL_Y_RANGE = 2

ODOMETRY_FREQ = 100  # in Hz
COLOR_FREQ = 100  # in Hz

def blender2real(x, y):
    """ Shift origin to bottom left corner """
    return x + BLENDER_X_RANGE/2, y + BLENDER_Y_RANGE/2


def acq_odometry_morse(odometry_queue, q_lock):
    with Morse() as simu:
        while True:
            # Get odometry data on morse
            odometry_data = simu.robot.odometry.get_local_data().result()
            vx = odometry_data['vx']
            vy = odometry_data['vx']
            total_speed = pow(pow(vx, 2) + pow(vy, 2), 0.5) * 1000 # to mm per sec
            theta_speed = odometry_data['wz']
            current_time = simu.time()
            # Write in queue to process
            q_lock.acquire()
            odometry_queue.put((total_speed, theta_speed, current_time))
            q_lock.release()

            # time.sleep(1/ODOMETRY_FREQ)


def acq_rgb_morse(rgb_queue, q_lock):
    image = misc.imread(TABLE_IMAGE_PATH)
    height = len(image)
    width = len(image[0])
    with Morse() as simu:
        while True:
            #video_data = simu.robot.videocamera.get_properties().result() # Doesn't work...

            # Get real robot position in order to get associated color
            pose = simu.robot.pose.get_local_data().result()
            x_blender = pose['x']
            y_blender = pose['y']
            (x, y) = blender2real(x_blender, y_blender)

            # Get supposing color based on real robot location
            color = image[height - int(y * height / REAL_Y_RANGE)][int(x * width/ REAL_X_RANGE)]

            # Write in queue to process
            q_lock.acquire()
            rgb_queue.put(tuple(color))
            q_lock.release()
            # time.sleep(1/COLOR_FREQ)
