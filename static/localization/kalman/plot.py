import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.collections import PatchCollection
import math

# To no re-plot a state if it has not move much
MINIMUM_REFRESH_DISTANCE = 0.01  # In meter
MINIMUM_REFRESH_TURN = 5 * np.pi / 180  # In radian


def create_plot_ax():
    ax = plt.gca()
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 2)
    return ax


def get_error_matrix_ellipse(matrix_error, state):

    # Compute eigenvalues and eigenvector of the matrix
    vals, vecs = np.linalg.eigh(matrix_error[:2, :2])
    # vals, vecs = np.linalg.eigh(np.array([[1000, 250], [250, 500]]))

    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]

    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * 3 * np.sqrt(vals)
    ellip = Ellipse(xy=(state.x, state.y), width=width, height=height, angle=theta, fill=True, alpha=0.3)
    # ellip = Ellipse(xy=(0, 0), width=width, height=height, angle=theta, fill=False, facecolor="#00ffff")

    return ellip


def plot_error_matrix(error_matrix, state, ax=plt.gca()):
    ellipse = get_error_matrix_ellipse(error_matrix, state)
    ax.add_artist(ellipse)


def plot_error_matrix_list(error_matrix_list, state_list):
    ellipses = []
    for i in range(len(error_matrix_list)):
        ellipse = get_error_matrix_ellipse(error_matrix_list[i], state_list[i])
        ellipses.append(ellipse)
    fig, ax = plt.subplots()
    p = PatchCollection(ellipses)
    ax.add_collection(p)
    plt.show()


def plot_state(state, color, ax=plt.gca()):
        x, y = state.x, state.y
        ax.plot(x, y, color + 'o')
        x_end = x + 0.1 * math.cos(state.theta)
        y_end = y + 0.1 * math.sin(state.theta)
        ax.plot([x, x_end], [y, y_end], color)


def plot_estimated_state(state, ax=plt.gca()):
    plot_state(state, 'r', ax)


def plot_actual_state(state, ax=plt.gca()):
    plot_state(state, 'g', ax)



def need_to_refresh(current_state, previous_state):
    """ Return True if we need to refresh plot between 2 states  """
    return (previous_state is None) or (current_state.distance(previous_state) > MINIMUM_REFRESH_DISTANCE or
                                        current_state.turn(previous_state) > MINIMUM_REFRESH_TURN)

