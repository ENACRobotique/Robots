import numpy as np
import math


class AbstractModel():

    def __init__(self, Q):
        """
        :param Q: Covariance matrix of the Gaussian noise of the model
        """
        self.Q = Q

    def transition_function(self, state, command):
        """
        :param state: Current state of the system (X_k)
        :param command: Command given to the system (U_k)
        :return: The new estimate of the system X_k+1
        """
        return state

    def F(self, state, command):
        """
        :param state: Current state of the system (X_k)
        :param command: Command given to the system
        :return: The jacobian of the transition function wrt the state X, evaluated in the current state and command
        """
        return np.eye(len(state))

    def G(self, state, command):
        """
        :param state: Current state of the system (X_k)
        :param command: Command given to the system (U_k)
        :return: The jacobian of the transition function wrt the command U, evaluated in the current state and command
        """
        return np.zeros((len(state), len(command)))

    def predict(self, state, command, matrix_error):
        """
        :param state: Current state (X_k)
        :param command: Command given (U_k)
        :param matrix_error : Current matrix error of the system (P_k)
        :return: (new estimated state X_k+1, new estimated matrix error P_k+1)
        """
        new_estimate = self.transition_function(state, command)

        # Check if we need to add noise
        # if command.has_noise():
        #     noise = self.Q
        # else:
        #     noise = np.zeros(self.Q.shape)
        noise_multiplicator = np.diag([command.total_speed * 3, command.rotation_speed * 2])
        noise = noise_multiplicator.dot(self.Q)

        F = self.F(state, command)
        G = self.G(state, command)
        new_matrix_error = np.dot(F, matrix_error).dot(np.transpose(F)) + np.dot(G, noise).dot(np.transpose(G))
        state.time = command.time
        return new_estimate, new_matrix_error


class OdometricModel(AbstractModel):

    def __init__(self, Q):
        super().__init__(Q)

    def transition_function(self, state, command):
        dt = command.time - state.time
        mean_theta = (state.theta + (command.rotation_speed * dt / 2)) #% (2 * np.pi)
        state.x += command.total_speed * dt * np.cos(mean_theta)
        state.y += command.total_speed * dt * np.sin(mean_theta)
        state.theta = (state.theta + command.rotation_speed * dt) #% (2 * np.pi)
        return state

    def F(self, state, command):
        dt = command.time - state.time
        mean_theta = (state.theta + (command.rotation_speed * dt / 2)) #% (2 * np.pi)

        F = np.zeros((3, 3))
        F[0] = [1, 0, - command.total_speed * dt * math.sin(mean_theta)]
        F[1] = [0, 1, command.total_speed * dt * math.cos(mean_theta)]
        F[2] = [0, 0, 1]

        return F

    def G(self, state, command):
        dt = command.time - state.time
        mean_theta = (state.theta + (command.rotation_speed * dt / 2)) #% (2 * np.pi)

        G = np.zeros((3, 2))
        G[0] = [dt * math.cos(mean_theta), - 0.5 * command.total_speed * pow(dt, 2) * math.sin(mean_theta)]
        G[1] = [dt * math.sin(mean_theta),  0.5 * command.total_speed * pow(dt, 2) * math.cos(mean_theta)]
        G[2] = [0, dt]

        return G

    def predict(self, state, command, matrix_error):
        return super().predict(state, command, matrix_error)
