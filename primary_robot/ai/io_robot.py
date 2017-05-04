from enum import *
import RPi.GPIO as GPIO

BALL_PICKER_MOTOR = 3
CANNON_MOTOR = 4

PIN_LED_RED = 22  # In GPIO.BOARD number
PIN_LED_GREEN = 24
PIN_LED_BLUE = 40
PIN_CORD = 16
PIN_COLOR = 18


class IO(object):
    def __init__(self, robot):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN_LED_RED, GPIO.OUT)
        GPIO.setup(PIN_LED_GREEN, GPIO.OUT)
        GPIO.setup(PIN_LED_BLUE, GPIO.OUT)
        GPIO.setup(PIN_CORD, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PIN_COLOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.robot = robot
        self._cord_state = None
        self._color = None
        self.led_color = None
        self.ball_picker_state = None
        self.cannon_state = None
        self.cannon_barrier_state = None
        self.rocket_launcher_state = None
        self.stop_ball_picker()
        self.stop_cannon()
        self.close_cannon_barrier()
        self.lock_rocket_launcher()
        self.set_led_color(self.LedColor.BLACK)
        self._read_cord(PIN_CORD)
        self._read_switch(PIN_COLOR)

    class LedColor(Enum):
        BLACK = (GPIO.LOW, GPIO.LOW, GPIO.LOW)
        RED = (GPIO.HIGH, GPIO.LOW, GPIO.LOW)
        GREEN = (GPIO.LOW, GPIO.HIGH, GPIO.LOW)
        BLUE = (GPIO.LOW, GPIO.LOW, GPIO.HIGH)
        YELLOW = (GPIO.HIGH, GPIO.HIGH, GPIO.LOW)
        PURPLE = (GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
        CYAN = (GPIO.LOW, GPIO.HIGH, GPIO.HIGH)
        WHITE = (GPIO.HIGH, GPIO.HIGH, GPIO.HIGH)

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

    @property
    def cord_state(self):
        self._read_cord(PIN_CORD)
        return self._cord_state

    @property
    def color(self):
        self._read_switch(PIN_COLOR)
        return self._color

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

    def set_led_color(self, color):
        GPIO.output(PIN_LED_RED, color.value[0])
        GPIO.output(PIN_LED_GREEN, color.value[1])
        GPIO.output(PIN_LED_BLUE, color.value[2])
        self.led_color = color

    def _led_init(self):
        GPIO.setup(PIN_LED_RED, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LED_GREEN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LED_BLUE, GPIO.OUT, initial=GPIO.LOW)

    def _read_cord(self, channel):
        if GPIO.input(channel):
            self._cord_state = self.CordState.IN
        else:
            self._cord_state = self.CordState.OUT

    def _read_switch(self, channel):
        if GPIO.input(channel):
            self._color = self.Color.BLUE
        else:
            self._color = self.Color.YELLOW

