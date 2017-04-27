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

    class RocketLauncherState(Enum):
        LOCKED = "locked"
        OPEN = "open"

    def __init__(self, robot):
        self.robot = robot
        self.cord_state = None
        self.color = None
        self.ball_picker_state = None
        self.cannon_state = None
        self.cannon_barrier_state = None
        self.rocket_launcher_state = None
        self.stop_ball_picker()
        self.stop_cannon()
        self.close_cannon_barrier()
        self.lock_rocket_launcher()

    def start_ball_picker(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.START_BALL_PICKER_MOTOR
        self.robot.communication.send_message(down_msg)
        self.ball_picker_state = self.BallPickerState.STARTED
        pass

    def stop_ball_picker(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.STOP_BALL_PICKER_MOTOR
        self.robot.communication.send_message(down_msg)
        self.ball_picker_state = self.BallPickerState.STOPPED

    def start_cannon(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.START_CANNON_MOTOR
        self.robot.communication.send_message(down_msg)
        self.cannon_state = self.CannonState.FIRING

    def stop_cannon(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.STOP_CANNON_MOTOR
        self.robot.communication.send_message(down_msg)
        self.cannon_state = self.CannonState.IDLE

    def close_cannon_barrier(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CLOSE_CANNON_BARRIER
        self.robot.communication.send_message(down_msg)
        self.cannon_barrier_state = self.CannonBarrierState.CLOSED

    def open_cannon_barrier(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.OPEN_CANNON_BARRIER
        self.robot.communication.send_message(down_msg)
        self.cannon_barrier_state = self.CannonBarrierState.OPEN

    def lock_rocket_launcher(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.LOCK_ROCKET_LAUNCHER
        self.robot.communication.send_message(down_msg)
        self.rocket_launcher_state = self.RocketLauncherState.LOCKED

    def open_rocket_launcher(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.OPEN_ROCKET_LAUNCHER
        self.robot.communication.send_message(down_msg)
        self.rocket_launcher_state = self.RocketLauncherState.OPEN