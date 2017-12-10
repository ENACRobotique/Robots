from threading import Thread, Lock
import queue
import numpy as np
import time
from state import State
from kalman_filter import KalmanFilter
import model
import sensor
import acquire_data
import plot
import copy
import matplotlib.pyplot as plt

PLOT_MODE = True  # Set to false to disable plotting
PLOT_RATE = 3  # Frequency of plot (Hz)

# Data receiving threads
data_queue = queue.Queue() # Contains tuple of the form (DATA_TYPE, data)
data_queue_lock = Lock()
# thread_odo = Thread(target=acquire_data.get_stimulated_odometry, args=(data_queue, data_queue_lock,))
thread_odo = Thread(target=acquire_data.get_morse_odometry, args=(data_queue, data_queue_lock,))
thread_odo.start()
thread_measurement1 = Thread(target=acquire_data.get_morse_measurement, args=(data_queue, data_queue_lock,))
thread_measurement1.start()


# Create motion model
Q = np.diag([0.005, 4 * np.pi / 180])  # Represent model uncertainties
odometry_model = model.OdometricModel(Q)

# Initialize kalman filter
initial_state = State(x=1.5, y=1, theta=0 * np.pi / 180, date=0)
initial_error_matrix = np.diag([0.0001, 0.0001, 0.0001])
kf = KalmanFilter(odometry_model, copy.copy(initial_state), initial_error_matrix)
current_state = initial_state
current_error = initial_error_matrix
previous_print_state = None
previous_print_error = None

# Plot
if PLOT_MODE:
    last_plot_time = 0
    ax = plot.create_plot_ax()
    plt.ion()

# Robot time
synchronized = False

while True:
    if data_queue.qsize() > 0:
        print("Q size :" + str(data_queue.qsize()))
        # Get the older data (command or measurement)
        data_queue_lock.acquire()
        data_type, data = data_queue.get()
        data_queue_lock.release()

        # Predict
        if data_type == acquire_data.DATA_TYPE.COMMAND:
            if synchronized:  # Need to synchronize clock before computing any prediction
                kf.predict(data)

                current_state = copy.deepcopy(kf.state)
                current_error = copy.deepcopy(kf.error_matrix)

            else:
                synchronized = True

            # Update state time
            kf.synchronize_clock(data.time)

        # Update
        elif data_type == acquire_data.DATA_TYPE.MEASUREMENT:
            kf.update(data)
            current_state = copy.deepcopy(kf.state)
            current_error = copy.deepcopy(kf.error_matrix)

        # Plot
        if PLOT_MODE:
            # if plot.need_to_refresh(current_state, previous_print_state) and time.time() - last_plot_time > 1 / PLOT_RATE:
            if time.time() - last_plot_time > 1 / PLOT_RATE:


                plot.plot_error_matrix(copy.deepcopy(kf.error_matrix), copy.deepcopy(kf.state), ax)
                plt.draw()

                last_plot_time = time.time()
                previous_print_state = current_state
                previous_print_error = current_error

                print(current_state)
                # print(current_error)

    if PLOT_MODE:
        plt.pause(0.01)
