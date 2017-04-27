from enum import *

BALL_PICKER_MOTOR = 3
CANNON_MOTOR = 4


class IO:

    class CordState(Enum):
        IN = "in"
        OUT = "out"

    class Color(Enum):
        YELLOW = "yellow"
        BLUE = "blue"

    class BallPickerState(Enum):
        STARTED = "started"
        STOPPED = "stopped"

    class CannonState(Enum):
        IDLE = "idle"
        FIRING = "firing"

    class CannonBarrierState(Enum):
        OPEN = "open"
        CLOSED = "close"

    def __init__(self, robot):
        self.robot = robot
        self.cord_state = None
        self.color = None
        self.ball_picker_state = None
        self.cannon_state = None
        self.stop_ball_picker()
        self.stop_cannon()

    def start_ball_picker(self):
        self.robot.communication.send_start_motor(BALL_PICKER_MOTOR)  #TODO : faire cette fonction
        self.ball_picker_state = self.BallPickerState.STARTED
        pass

    def stop_ball_picker(self):
        self.robot.communication.send_stop_motor(BALL_PICKER_MOTOR)  #TODO: faire cette fonction
        self.ball_picker_state = self.BallPickerState.STOPPED

    def start_cannon(self):
        self.robot.communication.send_start_motor(CANNON_MOTOR)  #TODO: faire cette fonction
        self.cannon_state = self.CannonState.FIRING

    def stop_cannon(self):
        self.robot.communication.send_stop_motor(CANNON_MOTOR)  # TODO: faire cette fonction
        self.cannon_state = self.CannonState.IDLE
