from scripts.particle_filter import *
from scripts.AcqData import *
from threading import Thread, Lock
import time, queue, numpy

PARTICLE_NUMBER = 2500

FORWARD_NOISE = 0.1
TURN_NOISE = 0.002

RESAMPLE_THRESHOLD = 1000

#particles = create_gaussian_particles([3000, 0, 2], [10, 10, 0.01], 2000)
particles = create_uniform_particles([0, 3000], [0, 2000], [0, 2 * np.pi], PARTICLE_NUMBER)

# Data receiving threads
odo_queue = queue.Queue()
odo_queue_lock = Lock()
thread_odo = Thread(target=acq_odometry_morse, args=(odo_queue, odo_queue_lock,))
thread_odo.start()


rgb_queue = queue.Queue()
rgb_queue_lock = Lock()
thread_rgb = Thread(target=acq_rgb_morse, args=(rgb_queue, rgb_queue_lock),)
thread_rgb.start()

current_time = 0
with open('particle_data.txt', 'w+') as f:
    while True:

        state_change = False # Indicate if predict, update or resample was done during  current loop

        t0 = time.time()
        # Move and Predict
        if odo_queue.qsize() > 0:
            # Shared queue
            odo_queue_lock.acquire()
            (total_speed, theta_speed, new_time) = odo_queue.get()
            odo_queue_lock.release()
            if current_time > 0:
                predict(particles, total_speed, theta_speed, [FORWARD_NOISE, TURN_NOISE], new_time - current_time)
            current_time = new_time

            state_change = True
        t1 = time.time()


        # Measure and update
        if rgb_queue.qsize() > 0:
            # print(rgb_queue.qsize())
            # Shared Queue
            rgb_queue_lock.acquire()
            color = rgb_queue.get()
            rgb_queue_lock.release()

            update_color(particles, Color(color))

            state_change = True

            t2 = time.time()

            # Resample
            # print(max([p.weight for p in particles]))
            # print(min([p.weight for p in particles]))
            if effective_particle_weight(particles) <= RESAMPLE_THRESHOLD:
                print("resampled")
                particles = resample(particles)

        t3 = time.time()

        if state_change:
            f.write(str(particles) + '\n')
        t4 = time.time()
        # print("predict : {}, update : {}, resample : {}, plot : {}".format(t1-t0, t2-t1, t3-t2, t4-t3))