from collections import namedtuple
from enum import *
import RPi.GPIO as GPIO
import threading
import smbus
import time
import Adafruit_TCS34725


BALL_PICKER_MOTOR = 3
CANNON_MOTOR = 4

PIN_LED_RED = 22  # In GPIO.BOARD number
PIN_LED_GREEN = 24
PIN_LED_BLUE = 40
PIN_CORD = 18
PIN_COLOR = 16

MIN_US_RANGE = 11 # Min us distance on which us data is considered ok


class RGBSensor:
    def __init__(self, red=0, blue=0, green=0, clear=0):
        self.red = red
        self.blue = blue
        self.green = green
        self.clear = clear


rgb_sensor = RGBSensor(0, 0, 0, 0)

UltraSoundSensor = namedtuple('ultra_sound_sensor', ['address', 'position'])
us_sensors = [UltraSoundSensor(0x70, "front_left"), UltraSoundSensor(0x71, "front_right"),
            UltraSoundSensor(0x77, "rear_left"), UltraSoundSensor(0x76, "rear_middle_left"), UltraSoundSensor(0x72, "rear_right")]  #Sets US sensors here !, empty list if no US is plugged

#us_sensors = [UltraSoundSensor(0x70, "front_left"), UltraSoundSensor(0x71, "front_right")]
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
        self.set_led_color(self.LedColor.WHITE)

    def InitializeIOs(self, robot):
        self._thread_us_reader = USReader()
        self._thread_us_reader.start()
        self._thread_rgb = RGBReader()
        self._thread_rgb.start()
        self.robot = robot
        self._cord_state = None
        self._button_state = None
        self.led_color = None
        self.trap_state = None
        self.sorter_state = None
        self.cutter_state = None
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

    @property
    def get_rgb_grayscale(self):
        """ Return the grayscale value given by the rgb sensor, which is between 0 and 255"""
        return ((rgb_sensor.red + rgb_sensor.blue + rgb_sensor.green) / 3.0) * 255 / 1024

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
        print("[IO] trap opened")

    def close_trap(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CLOSE_TRAP
        self.robot.communication.send_message(down_msg)
        self.sorter_state = self.TrapState.CLOSE
        print("[IO] trap closed")

    def sorter_collect_ball_1(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.SORTER_COLLECT_1
        self.robot.communication.send_message(down_msg)
        self.sorter_state = self.SorterState.COLLECT1
        print("[IO] sorter in collect 1 position")

    def sorter_collect_ball_2(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.SORTER_COLLECT_2
        self.robot.communication.send_message(down_msg)
        self.trap_state = self.SorterState.COLLECT2
        print("[IO] sorter in collect 2 position")

    def sorter_up(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.SORTER_UP
        self.robot.communication.send_message(down_msg)
        self.trap_state = self.SorterState.UP
        print("[IO] sorter in up position")

    def cutter_open(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CUTTER_OPEN
        self.robot.communication.send_message(down_msg)
        self.cutter_state = self.CutterState.OPEN
        print("[IO] cutter is open")

    def cutter_close(self):
        down_msg = self.robot.communication.sMessageDown()
        down_msg.message_type = self.robot.communication.eTypeDown.CUTTER_CLOSE
        self.robot.communication.send_message(down_msg)
        self.cutter_state = self.CutterState.CLOSE
        print("[IO] cutter is closed")

    def set_led_color(self, color):
        GPIO.output(PIN_LED_RED, color.value[0])
        GPIO.output(PIN_LED_GREEN, color.value[1])
        GPIO.output(PIN_LED_BLUE, color.value[2])
        self.led_color = color
        print("[IO] Led switched to {}".format(color))

    def collection(self):
        for _ in range(3):
            self.do_action_and_wait(self.robot.io.cutter_close, 1)
            self.do_action_and_wait(self.robot.io.open_trap, 2)
            self.do_action_and_wait(self.robot.io.close_trap, 1)
            self.do_action_and_wait(self.robot.io.sorter_collect_ball_2, 1)
            self.do_action_and_wait(self.robot.io.cutter_open, 1)
            self.do_action_and_wait(self.robot.io.cutter_close, 1)
            self.do_action_and_wait(self.robot.io.open_trap, 2)
            self.do_action_and_wait(self.robot.io.close_trap, 1)
            self.do_action_and_wait(self.robot.io.cutter_open, 0)
            self.do_action_and_wait(self.robot.io.sorter_up, 4)
            self.do_action_and_wait(self.robot.io.sorter_collect_ball_1, 1)



    def do_action_and_wait(self, action, duration):
        action()
        debut = time.time()
        while time.time() - debut < duration:
            pass

    def _led_init(self):
        GPIO.setup(PIN_LED_RED, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LED_GREEN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_LED_BLUE, GPIO.OUT, initial=GPIO.LOW)

    def _read_cord(self, channel):
        if GPIO.input(channel):
            self._cord_state = self.CordState.OUT
        else:
            self._cord_state = self.CordState.IN

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
                try:
                    self.i2c.write_byte_data(sensor.address, 0, 81)
                except Exception as e:
                    print("Can not write on sensor {0} : {1}".format(hex(sensor.address), e))
                    us_sensors.remove(sensor)
            time.sleep(0.070)
            for i, sensor in enumerate(us_sensors):
                try:
                    dst = self.i2c.read_word_data(sensor.address, 2) / 255
                    if dst > MIN_US_RANGE:
                        us_sensors_distance[us_sensors[i]] = dst
                except Exception as e:
                    print("Can not read on sensor {0} : {1}".format(hex(sensor.address), e))
                    us_sensors.remove(sensor)



class RGBReader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.i2c = smbus.SMBus(1)

    def run(self):
        tcs = Adafruit_TCS34725.TCS34725()
        tcs.set_interrupt(False)
        while True:
            r, g, b, c = tcs.get_raw_data()
            rgb_sensor.red = r
            rgb_sensor.blue = b
            rgb_sensor.green = g
            # _lux = Adafruit_TCS34725.calculate_lux(r, g, b)
            # print('Color: red={0} green={1} blue={2} clear={3}'.format(r, g, b, c))
            # Print out the lux.
            # print('Luminosity: {0} lux'.format(_lux))


