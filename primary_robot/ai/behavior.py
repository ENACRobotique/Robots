class Behavior:
    def __init__(self, robot):
        self.robot = robot

    def loop(self):
        raise NotImplementedError("This method must be overrided")
