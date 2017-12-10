MINIMUM_TOTAL_SPEED = 5e-4
MINIMUM_ROTATION_SPEED = 5e-8

class Command:
    def __init__(self, total_speed, rotation_speed, robot_time):
        self.total_speed = total_speed
        self.rotation_speed = rotation_speed
        self.time = robot_time

    def has_noise(self):
        """ Return True if we need to consider noise for this command, for example if the command is null (idle robot)
         we do not consider noise"""
        return (self.total_speed > MINIMUM_TOTAL_SPEED) or (self.rotation_speed > MINIMUM_ROTATION_SPEED)

    def __len__(self):
        return 2
