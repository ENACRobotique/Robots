""" Run this script to plot particles from last simulation """

import matplotlib.pyplot as plt
from scripts.particle_filter import *

# Plot
ax = plt.figure().add_subplot(111)
ax.set_xlim(0, 3000)
ax.set_ylim(0, 2000)
particles = []

with open('particle_data.txt') as f:
    line = f.readline()
    while len(line) > 1:
        particle_list_string = line[1:-2].split(',')
        part_nb = len(particle_list_string)
        particles = [Particle() for _ in range(part_nb)]
        for i in range(part_nb):
            (x, y, theta, weight) = particle_list_string[i][1:].split(' ')
            try:
                # particles.append(Particle(float(x), float(y), float(theta), float(weight)))
                particles[i] = Particle(float(x), float(y), float(theta), float(weight))
            except ValueError:
                print("error with x:{}, y:{}, theta:{}, w:{}".format(x, y, theta, weight))
        ax.clear()
        plot_particules_weight(particles, ax)
        ax.set_xlim(0, 3000)
        ax.set_ylim(0, 2000)
        plt.draw()
        plt.pause(0.001)
        line = f.readline()
