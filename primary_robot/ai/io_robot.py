from collections import namedtuple
from enum import *
import RPi.GPIO as GPIO
import threading
import smbus
import time

BALL_PICKER_MOTOR = 3
CANNON_MOTOR = 4

PIN_LED_RED = 22  # In GPIO.BOARD number
PIN_LED_GREEN = 24
PIN_LED_BLUE = 40
PIN_CORD = 18
PIN_COLOR = 16

UltraSoundSensor = namedtuple('ultra_sound_sensor', ['address', 'position'])
# us_sensors = [UltraSoundSensor(0x76, "front_left"), UltraSoundSensor(0x77, "front_right"), UltraSoundSensor(0x73, "rear_left"),
#               UltraSoundSensor(0x74, "rear_center"), UltraSoundSensor(0x72, "rear_right")]  #Sets US sensors here !, empty list if no US is plugged

us_sensors = []  #Sets US sensors here !, empty list if no US is plugged

#us_sensors=[]
us_sensors_distance = {us_sensors[i]: 0 for i in range(len(us_sensors))}


def get_us_distance(i):
    global us_sensors, us_sensors_distance
    return us_sensors_distance[us_sensors[i]]


class IO(object):
    def __init__(self, robot):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN_LED_RED, GPIO.OUT)
        GPIO.setup(PIN_LED_GREEN, GPIO.OUT)
        GPIO.setup(PIN_LED_BLUE, GPIO.OUT)
        GPIO.setup(PIN_CORD, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PIN_COLOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self._thread_us_reader = USReader()
        self._thread_us_reader.start()
        self.robot = robot
        self._cord_state = None
        self._button_state = None
        self.led_color = None
        self.trap_state = None
        self.sorter_state = None
        self.cutter_state = None
        self.set_led_color(self.LedColor.BLACK)
        self._read_cord(PIN_CORD)
        self._read_switch(PIN_COLOR)
        self.open_trap()
        self.close_trap()
        self.sorter_collect_ball_1()
        self.sorter_collect_ball_2()
        self.sorter_up()

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

    class ButtonState(Enum):
        PRESSED = "pressed"
        RELEASED = "released"

    class TrapState(Enum):
        OPEN = "open"
        CLOSE = "close"

    class SorterState(Enum):
        COLLECT1 = "collecting 1"
        COLLECT2 = "collecting 2"
        UP = "up"

    class CutterState(Enum):
        OPEN = "open"
        CLOSE = "close"


    @property
    def cord_state(self):
        self._read_cord(PIN_CORD)
        return self._cord_state

    @property
    def button_state(self):
        self._read_switch(PIN_COLOR)
        return self._button_state

    @staticmethod
    def get_us_distance_by_postion(position):
        global us_sensors
        correct_sensors = [i for i in range(len(us_sensors)) if position.lower() in us_sensors[i].position.lower()]
        distances = [get_us_distance(i) for i in correct_sensors]
        if len(distances) == 0:
            return 500000
        else:
            return min(distances)

    @property
    def front_distance(self):
        return self.get_us_distance_by_postion("front")

    @property
    def rear_distance(self):
        return self.get_us_distance_by_postion("rear")


    # def start_cannon(self, speed=55):
    #     down_msg = self.robot.communication.sMessageDown()
    #     down_msg.message_type = self.robot.communication.eTypeDown.START_CANNON_MOTOR
    #
    #     #FIXME : Each time the following two lines are called,
    #     #FIXME : Satan comes a bit closer to our world
    #     #FIXME : (no really it should be fixed by defining own structure to this action)
    #     down_msg.payload = self.robot.communication.sTrajectory()
    #     down_msg.payload.speed = speed - 128 #Hack to set the speed between 0-255 (and not -128, 127)
    #
    #     self.robot.communication.send_message(down_msg)
    #     self.cannon_state = self.CannonState.FIRING
    #     if __debug__:
    #         print("[IO] cannon started")

    def open_trap(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.OPEN_TRAP
        self.robot.communication.send_message(down_msg)
        self.sorter_state = self.TrapState.OPEN
        if __debug__:
            print("[IO] trap opened")

    def close_trap(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CLOSE_TRAP
        self.robot.communication.send_message(down_msg)
        self.sorter_state = self.TrapState.CLOSE
        if __debug__:
            print("[IO] trap closed")

    def sorter_collect_ball_1(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.SORTER_COLLECT_1
        self.robot.communication.send_message(down_msg)
        self.sorter_state = self.SorterState.COLLECT1
        if __debug__:
            print("[IO] sorter in collect 1 position")

    def sorter_collect_ball_2(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.SORTER_COLLECT_2
        self.robot.communication.send_message(down_msg)
        self.trap_state = self.SorterState.COLLECT2
        if __debug__:
            print("[IO] sorter in collect 2 position")

    def sorter_up(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.SORTER_UP
        self.robot.communication.send_message(down_msg)
        self.trap_state = self.SorterState.UP
        if __debug__:
            print("[IO] sorter in up position")

    def cutter_open(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CUTTER_OPEN
        self.robot.communication.send_message(down_msg)
        self.cutter_state = self.CutterState.OPEN
        if __debug__:
            print("[IO] cutter is open")

    def cutter_close(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CUTTER_CLOSE
        self.robot.communication.send_message(down_msg)
        self.cutter_state = self.CutterState.CLOSE
        if __debug__:
            print("[IO] cutter is closed")

    def set_led_color(self, color):
        GPIO.output(PIN_LED_RED, color.value[0])
        GPIO.output(PIN_LED_GREEN, color.value[1])
        GPIO.output(PIN_LED_BLUE, color.value[2])
        self.led_color = color
        if __debug__:
            print("[IO] Led switched to {}".format(color))

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
            self._button_state = self.ButtonState.RELEASED
        else:
            self._button_state = self.ButtonState.PRESSED


class USReader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.i2c = smbus.SMBus(1)

    def run(self):
        global us_sensors, us_sensors_distance
        while True:
            for i, sensor in enumerate(us_sensors):
                self.i2c.write_byte_data(sensor.address, 0, 81)
            time.sleep(0.070)
            for i, sensor in enumerate(us_sensors):
                dst = self.i2c.read_word_data(sensor.address, 2) / 255
                if dst != 0:
                    us_sensors_distance[us_sensors[i]] = dst


