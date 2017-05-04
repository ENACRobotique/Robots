import bitstring
from enum import *
import serial
import time
from collections import deque

SERIAL_BAUDRATE = 115200
SERIAL_PATH = "/dev/ttyAMA0"
SERIAL_SEND_TIMEOUT = 500  # ms

DOWN_MSG_SIZE = 47
UP_MSG_SIZE = 9

### Up (Prop -> raspi) message declaration ###


class eTypeUp(Enum):
    ACK = 0
    NON_ACK = 1
    POINT_REACHED = 2
    POSITION = 3


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
        self.theta = None  # orientation in degree/radians, in table frame
        self.point_id = None  # the id of the trajectory point reached. Used in POINT_REACHED.

    def deserialize(self, packed):
        s = bitstring.BitStream(packed)
        type_int, self.down_id, self.x, self.y, self.theta, self.point_id = s.unpack(
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
        self._serial_port = serial.Serial(serial_path, baudrate)
        self._current_msg_id = 0  # type: int
        self._mailbox = deque()

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
        msg.id = self._current_msg_id
        self._current_msg_id = (self._current_msg_id + 1) % 256
        serialized = msg.serialize().tobytes()
        for i in range(max_retries):
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
        # 2017 Cup Specials
        START_BALL_PICKER_MOTOR = 4
        STOP_BALL_PICKER_MOTOR = 5
        START_CANNON_MOTOR = 6
        STOP_CANNON_MOTOR = 7
        OPEN_CANNON_BARRIER = 8
        CLOSE_CANNON_BARRIER = 9
        OPEN_ROCKET_LAUNCHER = 10
        LOCK_ROCKET_LAUNCHER = 11

    class sTrajElement():
        def __init__(self):
            self.x = 0  # :16

            self.y = 0  # :16

        def serialize(self):
            return bitstring.pack('uintle:16, uintle:16', self.x, self.y)

    class sTrajectory():
        def __init__(self):
            self.nb_traj = 0  # :8

            self.speed = 0  # :8

            self.theta_final = 0  # :16

            self.element = []

        def serialize(self):
            ser = bitstring.pack('uint:8, uint:8, uintle:16', self.nb_traj, self.speed, self.theta_final)

            for elt in self.element:
                ser += elt.serialize()

            return ser

    class sRepositionning():
        def __init__(self):
            self.x = 0  # 16

            self.y = 0  # 16

            self.theta = 0  # 16

        def serialize(self):
            return bitstring.pack('uintle:16, uintle:16, uintle:16', self.x, self.y, self.theta)

    class sMessageDown():
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
