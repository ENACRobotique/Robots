class KalmanFilter():

    def __init__(self, model, initial_state, initial_error_matrix):
        self.model = model
        self.state = initial_state
        self.error_matrix = initial_error_matrix

    def predict(self, command):
        new_state, new_error_matrix = self.model.predict(self.state, command, self.error_matrix)
        self.state = new_state
        self.error_matrix = new_error_matrix

    def update(self, measurement):
        new_state, new_error_matrix = measurement.sensor.update(self.state, measurement, self.error_matrix)
        self.state = new_state
        self.error_matrix = new_error_matrix

    def synchronize_clock(self, robot_time):
        self.state.time = robot_time
