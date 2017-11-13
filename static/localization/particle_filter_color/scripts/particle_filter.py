import numpy as np
from numpy.random import uniform, randn
from enum import Enum
import matplotlib.pyplot as plt


WHEEL_BASE = 300  # distance in mm between the two wheels

# Colors
class Color(Enum):
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    BLACK = (0, 0, 0)


# Particle
class Particle(object):
    def __init__(self, x=0, y=0, theta=0, weight=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    # Called when the robot move
    def move(self, total_speed, theta_speed, dt, forward_noise, turn_noise):
        mean_theta = (self.theta + (theta_speed * dt / 2)) % (2 * np.pi)
        self.x += total_speed * dt * np.cos(mean_theta) + (np.random.normal() * forward_noise)
        self.y += total_speed * dt * np.sin(mean_theta) + (np.random.normal() * forward_noise)
        self.theta += (theta_speed * dt + (np.random.normal() * turn_noise)) % (2 * np.pi)

    def __str__(self):
        return "{0:.0f} {1:.0f} {2:.3f} {3:.5f}".format(self.x, self.y, self.theta, self.weight)

    def __repr__(self):
        return self.__str__()

    def copy(self):
        return Particle(self.x, self.y, self.theta, self.weight)


# Plots
def plot_particules_heading(particules, ax):
    for p in particules:
        ax.quiver(p.x, p.y, np.cos(p.theta), np.sin(p.theta), units='xy', width=8, scale=0.01)
        #plt.scatter(p.x, p.y, marker=(3, 0 , p.theta * 180 / np.pi))
        plt.axis('equal')


def plot_particules_weight(particules, ax):
    # ax.scatter([p.x for p in particules], [p.y for p in particules], s=[p.weight * len(particules) * 10 for p in particules])
    particule_number = len(particules)
    x = [0 for _ in range(particule_number)]
    y = [0 for _ in range(particule_number)]
    s = [0 for _ in range(particule_number)]
    for p in particules:
        x.append(p.x)
        y.append(p.y)
        s.append(p.weight * 10000)
    ax.scatter(x, y, s=s)


# Initialization : Gaussian or uniform distribution
def create_uniform_particles(x_range, y_range, hdg_range, N):
    x = uniform(x_range[0], x_range[1], size=N)
    y = uniform(y_range[0], y_range[1], size=N)
    theta = uniform(hdg_range[0], hdg_range[1], size=N)
    particles = [Particle(x[i], y[i], theta[i], 1/N) for i in range(N)]
    return particles

def create_gaussian_particles(mean, std, N):
    x = mean[0] + (randn(N) * std[0])
    y = mean[1] + (randn(N) * std[1])
    theta = mean[2] + (randn(N) * std[2])
    particles = [Particle(x[i], y[i], theta[i] % (2 * np.pi), 1/N) for i in range(N)]
    return particles


# Predict, update and resample
def predict(particles, total_speed, theta_speed, noise, dt):
    """
    Predict new particles states (with taking uncertainty into account)
    :param particles: particles - 1*N
    :param total_speed: total velocity in mm per sec
    :param theta_speed: differentiate rotation in rad/sec
    :param noise: Command std noise - 1*2 (forward_noise, turn_noise)
    :param dt: elapsed time since last prediction
    :return:
    """

    # Odometry command
    # left_speed = command[0]
    # right_speed = command[1]

    # Useful calculation
    # total_speed = (right_speed + left_speed) / 2 # mm per sec
    # theta_speed = (right_speed - left_speed) / WHEEL_BASE # rad per sec

    # Update state for each particle (using odometry + noise)
    for p in particles:
        p.move(total_speed, theta_speed, dt, noise[0], noise[1])


def update_color(particles, color):
    weight_sum = 0
    # Attribute new weight
    for p in particles:

        p.weight += 0.001  # To avoid round-off to zero?
        # TODO :  Be less a pig
        if color == Color.BLUE:
            p.weight *= int((0 <= p.x <= 1500) and (0 <= p.y <= 1000)) * 10
        elif color == Color.RED:
            p.weight *= int((0 < p.x <= 1500) and (1000 <= p.y <= 2000)) * 10
        elif color == Color.GREEN:
            p.weight *= int((1500 < p.x <= 3000) and (1000 < p.y <= 2000)) * 10
        elif color == Color.BLACK:
            p.weight *= int((1500 <= p.x <= 3000) and (0 < p.y <= 1000)) * 10
        else:
            print("Color {} not found".format(color))

        # Check first if particle is still inside the table
        if p.x < 0 or p.x > 3000 or p.y < 0 or p.y > 2000:
            p.weight = 1e-30

        weight_sum += p.weight
    # Normalize
    for p in particles:
        p.weight /= weight_sum


def resample(particles):

    # TODO : check if weight sum equals 1

    cumulative_sum = np.cumsum([p.weight for p in particles])
    # Randomly pick N number between 0 and 1, then associate each number the closest index weight
    resampled_indexes = np.searchsorted(cumulative_sum, np.random.random(len(particles)))

    return [particles[i].copy() for i in resampled_indexes]


# Used to quantify how much particle are effective (i.e have high weigh)
def effective_particle_weight(particles):
    return 1/np.sum(np.square([p.weight for p in particles]))

