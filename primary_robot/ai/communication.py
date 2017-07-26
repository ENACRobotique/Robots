import bitstring
from enum import *
import serial
import time
from collections import deque

from RPi import GPIO

SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
SERIAL_SEND_TIMEOUT = 500  # ms

DOWN_MSG_SIZE = 47
UP_MSG_SIZE = 9

RAD_TO_UINT16_FACTOR = 10430.378350470453
SPEED_TO_UINT8_SUBSTRACTOR = 127
PIN_RESET_TEENSY = 33  # in GPIO board number


### Up (Prop -> raspi) message declaration ###


class eTypeUp(Enum):
    ACK = 0
    NON_ACK = 1
    POINT_REACHED = 2
    POSITION = 3
    POINTS_BUFFER_FULL = 4
    RECALAGE_OK = 5


class sMessageUp:
    """
    Class defining the up (teensy -> raspi) messages
    :type type: eTypeUp
    :type down_id: int
    :type x: int
    :type y: int
    :type theta: float
    :type point_id: int
    """

    def __init__(self):
        self.type = None  # The type of the message
        self.down_id = None  # The id of the answered down (raspi-> teensy) message. Used for ACK, NON_ACK and POINT_REACHED
        self.x = None  # x position in mm, in table frame
        self.y = None  # y position in mm, in table frame
        self.__theta = None  # orientation in degree/radians, in table frame
        self.point_id = None  # the id of the trajectory point reached. Used in POINT_REACHED.

    @property
    def theta(self):
        return self.__theta / RAD_TO_UINT16_FACTOR

    @theta.setter
    def theta(self, theta):
        self.__theta = theta * RAD_TO_UINT16_FACTOR

    def deserialize(self, packed):
        s = bitstring.BitStream(packed)
        type_int, self.down_id, self.x, self.y, self.__theta, self.point_id = s.unpack(
            'uint:8, uint:8, uintle:16, uintle:16, uintle:16, uint:8')
        self.type = eTypeUp(type_int)


# End up message declaration


class Communication:
    def __init__(self, serial_path=SERIAL_PATH, baudrate=SERIAL_BAUDRATE):
        """
        ctor of the communication class
        :param serial_path: The path of the serial file
        :type serial_path: str
        :param baudrate: The baudrate of UART (must the same as the one on the other board)
        :type baudrate: int
        """
        self.mock_communication = True  # Set to True if Serial is not plugged to the Teensy
        try:
            self._serial_port = serial.Serial(serial_path, baudrate)
        except Exception as e:
            print(e)
        self._current_msg_id = 0  # type: int
        self._mailbox = deque()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN_RESET_TEENSY, GPIO.OUT)
        GPIO.output(PIN_RESET_TEENSY, GPIO.HIGH)
        self.reset_teensy()

    def reset_teensy(self):
        GPIO.output(PIN_RESET_TEENSY, GPIO.LOW)
        time.sleep(1)
        GPIO.output(PIN_RESET_TEENSY, GPIO.HIGH)
        time.sleep(10)

    def send_message(self, msg, max_retries=1000):
        """
        Send message via Serial (defined during the instantiation of the class)
        :param msg: the message to send
        :type msg: sMessageDown
        :param max_retries: the maximum number of resend (on timeout = SERIAL_SEND_TIMEOUT or on NON_ACK) before failing
        :type max_retries: int
        :return: 0 if the message is sent, -1 if max_retries has been reached
        :rtype: int
        """
        if self.mock_communication:
            max_retries = 0

        msg.id = self._current_msg_id
        self._current_msg_id = (self._current_msg_id + 1) % 256
        serialized = msg.serialize().tobytes()
        for i in range(max_retries):
            # print(serialized)
            self._serial_port.write(serialized)
            time_sent = int(round(time.time() * 1000))
            while self._serial_port.in_waiting < UP_MSG_SIZE:
                if int(round(time.time() * 1000)) - time_sent > SERIAL_SEND_TIMEOUT:
                    break  # waiting for ack
            if self._serial_port.in_waiting >= UP_MSG_SIZE:
                packed = self._serial_port.read(UP_MSG_SIZE)
                up_msg = sMessageUp()
                up_msg.deserialize(packed)
                if up_msg.type == eTypeUp.ACK:
                    return 0  # success
                elif up_msg.type == eTypeUp.POINT_REACHED or up_msg.type == eTypeUp.POSITION:
                    self._mailbox.append(up_msg)  # if it is not an ACK or a NONACK, store it to deliver later
        return -1  # failure

    def check_message(self):
        """
        Check if there is any incoming message on the Serial (defined during the instantiation of the class)
        and returns the oldest message.
        :return: The oldest message non read
        :rtype: sMessageUp
        """
        if self.mock_communication:
            return None

        if self._serial_port.in_waiting >= UP_MSG_SIZE:
            packed = self._serial_port.read(UP_MSG_SIZE)
            up_msg = sMessageUp()
            up_msg.deserialize(packed)
            self._mailbox.append(up_msg)
        if len(self._mailbox) > 0:
            return self._mailbox.popleft()
        return None

    ### Down (raspi -> prop) message declaration ###
    class eTypeDown(Enum):
        TRAJECTORY = 0
        STOP = 1
        RESTART = 2
        REPOSITIONING = 3
        EMPTY_POINTS = 4
        DO_RECALAGE = 5
        RESET = 6
        # 2017 Cup Specials
        START_BALL_PICKER_MOTOR = 7
        STOP_BALL_PICKER_MOTOR = 8
        START_CANNON_MOTOR = 9
        STOP_CANNON_MOTOR = 10
        OPEN_CANNON_BARRIER = 11
        CLOSE_CANNON_BARRIER = 12
        OPEN_ROCKET_LAUNCHER = 13
        LOCK_ROCKET_LAUNCHER = 14

    class sTrajElement:
        def __init__(self):
            self.x = 0  # :16

            self.y = 0  # :16

        def serialize(self):
            return bitstring.pack('uintle:16, uintle:16', self.x, self.y)

    class sTrajectory:
        def __init__(self):
            self.nb_traj = 0  # :8

            self.__speed = 0  # :8

            self.__theta_final = 0  # :16

            self.element = []

        @property
        def theta_final(self):
            return self.__theta_final / RAD_TO_UINT16_FACTOR

        @theta_final.setter
        def theta_final(self, theta_final):
            self.__theta_final = theta_final * RAD_TO_UINT16_FACTOR

        @property
        def speed(self):
            return self.__speed - SPEED_TO_UINT8_SUBSTRACTOR

        @speed.setter
        def speed(self, speed):
            self.__speed = speed + SPEED_TO_UINT8_SUBSTRACTOR

        def serialize(self):
            ser = bitstring.pack('uint:8, uint:8, uintle:16', self.nb_traj, self.__speed, self.__theta_final)

            for elt in self.element:
                ser += elt.serialize()

            return ser

    class sRepositionning:
        def __init__(self):
            self.x = 0  # 16

            self.y = 0  # 16

            self.__theta = 0  # 16

        @property
        def theta(self):
            return self.__theta / RAD_TO_UINT16_FACTOR

        @theta.setter
        def theta(self, theta):
            self.__theta = theta * RAD_TO_UINT16_FACTOR

        def serialize(self):
            return bitstring.pack('uintle:16, uintle:16, uintle:16', self.x, self.y, self.__theta)

    class sMessageDown:
        def __init__(self):

            self.id = 0  # :8

            self.message_type = None  # :8

            self.checksum = 0  # :8

            self.payload = None  # sReposionning or sTrajectory

        def serialize(self):

            ser2 = None
            if self.payload is not None:
                ser2 = self.payload.serialize()

                self.checksum = 0

                for octet in ser2.tobytes():
                    self.checksum += octet

            self.checksum = self.checksum % 0xFF

            ser = bitstring.pack('uint:8, uint:8, uint:8', self.id, self.message_type.value, self.checksum)
            serialized_msg = ser + ser2
            pad = bitstring.pack('pad:{}'.format((DOWN_MSG_SIZE - len(serialized_msg.tobytes())) * 8))
            return serialized_msg + pad


            ### End down message declaration ###
