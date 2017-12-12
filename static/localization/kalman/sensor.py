import numpy as np

class BasicSensor():
    def __init__(self, R):
        """
        :param R: Covariance matrix of the Gaussian noise of the sensor
        """
        self.R = R

    def expected_measurement(self, state):
        """
        :param state : Current estimated state
        :return: Expected value of the sensor reading based on estimated state (if the sensor is supposed to give
        directly the current state, just return the given state)
        """
        return state

    def H(self, state):
        """
        :param state : Current estimated state
        :return: World frame to sensor frame matrix (Identity if same frame)
        """
        return np.eye(len(state))

    def innovation(self, expected_measurement, measurement):
        angle_difference = (measurement.theta - expected_measurement.theta) % (2 * np.pi)
        if angle_difference >= np.pi:
            angle_difference -= 2 * np.pi
        innovation = np.array([measurement.x - expected_measurement.x, measurement.y - expected_measurement.y,
                               angle_difference])
        innovation.shape = (3, 1)  # To turn it into a column vector instead of a row vector
        return innovation

    def update(self, state, measurement, matrix_error):

        H = self.H(state)

        # Compute innovation
        innovation = self.innovation(self.expected_measurement(state), measurement)
        innovation_covariance = H.dot(matrix_error).dot(np.transpose(H)) + self.R

        # Compute kalman gain K
        kalman_gain = matrix_error.dot(np.transpose(H)).dot(np.linalg.inv(innovation_covariance))

        # Compute post state estimate
        post_state = state + kalman_gain.dot(innovation)

        # Compute post error covariance matrix
        post_matrix_error = (np.eye(len(state)) - kalman_gain.dot(H)).dot(matrix_error)

        return post_state, post_matrix_error

