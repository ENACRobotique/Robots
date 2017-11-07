from scripts.particle_filter import *
from scripts.AcqData import *
from threading import Thread, Lock
import time, queue

PARTICLE_NUMBER = 1500

FORWARD_NOISE = 0.1
TURN_NOISE = 0.01

RESAMPLE_THRESHOLD = 0 # Useless (resample at every step for now)

#particles = create_gaussian_particles([3000, 0, 2], [10, 10, 0.01], 2000)
particles = create_uniform_particles([0, 3000], [0, 2000], [0, 2 * np.pi], PARTICLE_NUMBER)

# Plot
ax = plt.figure().add_subplot(111)
ax.set_xlim(0, 3000)
ax.set_ylim(0, 2000)

# Data receiving threads
odo_queue = queue.Queue()
odo_queue_lock = Lock()
thread_odo = Thread(target=acq_odometry_morse, args=(odo_queue, odo_queue_lock,))
thread_odo.start()


rgb_queue = queue.Queue()
rgb_queue_lock = Lock()
thread_rgb = Thread(target=acq_rgb_morse, args=(rgb_queue, rgb_queue_lock),)
thread_rgb.start()

while True:
    # t0 = time.time()
    # Move and Predict
    if odo_queue.qsize() > 0:
        # print(odo_queue.qsize())
        # Shared queue
        odo_queue_lock.acquire()
        (total_speed, theta_speed) = odo_queue.get()
        odo_queue_lock.release()

        predict(particles, total_speed, theta_speed, [FORWARD_NOISE, TURN_NOISE], 0.3)
        # t1 = time.time()


    # Measure and update
    if rgb_queue.qsize() > 0:
        # print(rgb_queue.qsize())
        # Shared Queue
        rgb_queue_lock.acquire()
        color = rgb_queue.get()
        rgb_queue_lock.release()


        update_color(particles, Color(color))
        # t2  =time.time()

    #Resample
    if effective_particle_weight(particles) >= RESAMPLE_THRESHOLD:
        # print("resample")
        # print(effective_particle_weight(particles))
        particles = resample(particles)
    #t3 = time.time()

    ax.clear()
    plot_particules_weight(particles, ax)
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2000)
    plt.draw()
    plt.pause(0.001)
    # print("predict : {}, update : {}, resample : {}".format(t1-t0, t2-t1, t3-t2))