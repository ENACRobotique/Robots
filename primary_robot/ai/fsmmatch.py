from enum import Enum

import time

import math

from behavior import Behavior

END_MATCH_TIME = 100  # in seconds
INITIAL_WAIT = 0 #in seconds

#2017 specific
SMALL_CRATER_COLLECT_DURATION = 4  # in seconds
SMALL_CRATER_FIRE_DURATION = 4  # in seconds
GREAT_CRATER_COLLECT_DURATION = 1  # in seconds
STANDARD_SEPARATION_US = 20  # in cm
FULL_SPEED_CANNON_TIME = 2  # in seconds
AFTER_SEESAW_RECALAGE_MAX_TIME = 6  # in sec
FIRE1_RECALAGE_MAX_TIME = 3  # in sec
FIRE1_CANNON_POWER = 45  # between 0 and 255
MAX_CANNON_POWER = 105  # between 0 and 255
CANNON_AUGMENTATION_DISTANCE_STEP = 50  # in mm
CANNON_AUGMENTATION_POWER_STEP = 10  # between 0 and 255
WHITE_GRAYSCALE = 220 # between 0 and 255

# Coordinates
ROBOT_BACK2FRONT = 250
BUMPER_OFFSET = 45
Y_START = 243
Y_CROSSING = 1300
X_SWITCH_GREEN = 1870
THETA_INIT_GREEN = math.pi


class Color(Enum):
    # BLUE = "blue"
    # YELLOW = "yellow"
    ORANGE = "orange"
    GREEN = "green"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self.funny_action_finished = False
        self.state = StateColorSelection(self)

    def loop(self):
        time_now = time.time()
        if self.start_time is not None and time_now - self.start_time >= END_MATCH_TIME and self.state.__class__ != StateEnd:
            if __debug__:
                print("[FSMMatch] End match")
            next_state = StateEnd
        else:
            next_state = self.state.test()
        if next_state is not None:
            if __debug__:
                print("[FSMMatch] Leaving {}, entering {}".format(self.state.__class__.__name__, next_state.__name__))
            self.state.deinit()
            self.state = next_state(self)

    def start_match(self):
        if __debug__:
            print("[FSMMatch] Match Started")
        self.start_time = time.time()


class FSMState:
    def __init__(self, behavior):
        raise NotImplementedError("this state is not defined yet")

    def test(self):
        raise NotImplementedError("test of this state is not defined yet !")

    def deinit(self):
        raise NotImplementedError("deinit of this state is not defined yet !")


class StateInitialWait(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.start_time = time.time()

    def test(self):
        if time.time() - self.start_time > INITIAL_WAIT:
            if self.behavior.color == Color.ORANGE:
                return StateBeginOrange
            else:
                return StateBeginGreen

    def deinit(self):
        pass


class StateBeginOrange(FSMState):

    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.reposition_robot(44.75, Y_START, 0)#Attention origine Robot = position du point entre les 2 roues

        self.num_pos=0
        self.p1 = self.behavior.robot.locomotion.PointOrient(1300, Y_START + 100, 0)
        self.p2 = self.behavior.robot.locomotion.PointOrient(1130+45, 500, 3*math.pi/2)#Le bumper est décallé de 45mm du centre
        self.behavior.robot.locomotion.go_to_orient_point(self.p1, 50)#Avant

        self.stopped = False

    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished:
            if self.num_pos == 0:
                self.num_pos += 1
                self.behavior.robot.locomotion.go_to_orient_point(self.p2, -50)#Arrière
            else:
                return StateCloseSwitch

        if self.num_pos == 0:
            if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
                self.behavior.robot.locomotion.stop_robot()
                self.stopped = True

            if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
                self.behavior.robot.locomotion.restart_robot()
                self.stopped = False

        else:
            if self.behavior.robot.io.rear_distance <= STANDARD_SEPARATION_US and not self.stopped:
                self.behavior.robot.locomotion.stop_robot()
                self.stopped = True

            if self.behavior.robot.io.rear_distance > STANDARD_SEPARATION_US and self.stopped:
                self.behavior.robot.locomotion.restart_robot()
                self.stopped = False


    def deinit(self):
        pass


class StateBeginGreen(FSMState):

    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.reposition_robot(3000-44.75, Y_START, THETA_INIT_GREEN)  # Attention origine Robot = position du point entre les 2 roues

        self.num_pos = 0
        p1 = self.behavior.robot.locomotion.Point(2500, Y_START)
        p2 = self.behavior.robot.locomotion.Point(1700, Y_START+100)
        self.p3 = self.behavior.robot.locomotion.PointOrient(X_SWITCH_GREEN + BUMPER_OFFSET, 370,
                                                             3 * math.pi / 2)  # Le bumper est décallé de 45mm du centre
        self.behavior.robot.locomotion.follow_trajectory([p1, p2], THETA_INIT_GREEN, 100)

        self.stopped = False

    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished:
            if self.num_pos == 0:
                self.num_pos += 1
                self.behavior.robot.locomotion.go_to_orient_point(self.p3, -100)  # Arrière
            else:
                return StateCloseSwitch

        if self.num_pos == 0:
            if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
                self.behavior.robot.locomotion.stop_robot()
                self.stopped = True

            if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
                self.behavior.robot.locomotion.restart_robot()
                self.stopped = False
        else:
            if self.behavior.robot.io.rear_distance <= STANDARD_SEPARATION_US and not self.stopped:
                self.behavior.robot.locomotion.stop_robot()
                self.stopped = True
            if self.behavior.robot.io.rear_distance > STANDARD_SEPARATION_US and self.stopped:
                self.behavior.robot.locomotion.restart_robot()
                self.stopped = False

    def deinit(self):
        pass


class StateCloseSwitch(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.wait_for_repositionning = False
        self.recalage_start_time = 0

    def test(self):
        if not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True
            self.recalage_start_time = time.time()

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            self.behavior.robot.locomotion.reposition_robot(1880, 1780, 0.5 * math.pi)
            return self.exit()

        if self.wait_for_repositionning and not self.behavior.robot.locomotion.is_recalage_ended and time.time() - self.recalage_start_time > AFTER_SEESAW_RECALAGE_MAX_TIME:
            self.behavior.robot.locomotion.reposition_robot(1880, 1780, 0.5 * math.pi)
            return self.exit()

    def deinit(self):
        pass

    def exit(self):
        # if self.behavior.color == Color.ORANGE:
        #     return StateOrangeCrossing
        # else:
        #     return StateGreenCrossing
        return StateEnd


class StateGoRecupOrange(FSMState):

    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.reposition_robot(1130+45, 220.25, 3*math.pi/2)  # Attention origine Robot = position du point entre les 2 roues

        self.p1 = self.behavior.robot.locomotion.Point(1130 + 45, 500)
        self.p2 = self.behavior.robot.locomotion.Point(3000 - 300, 2000 - 190)
        #self.behavior.robot.locomotion.follow_trajectory([self.p1, self.p2], math.pi,  -75)  # Arrière
        self.behavior.robot.locomotion.follow_trajectory([self.p1], math.pi,  -75)  # Arrière

        self.stopped = False
        self.wait_for_repositionning = False

    def test(self):
        if time.time() - self.behavior.start_time > 80:
            return StateEnd

        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning :
            return StateEnd
            #TODO recalage arr
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            #TODO Recup Orange
            return StateEnd

        #Ultrasons
        if self.behavior.robot.io.rear_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True

        if self.behavior.robot.io.rear_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

    def deinit(self):
        pass

class StateColorSelection(FSMState):
    class ColorState(Enum):
        IDLE = "idle"
        PRESSED = "pressed"

    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.color = Color.ORANGE
        self.state = self.ColorState.IDLE
        self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.RED)

    def test(self):
        if self.state == self.ColorState.IDLE and self.behavior.robot.io.button_state == self.behavior.robot.io.ButtonState.PRESSED:
            if self.behavior.color == Color.ORANGE:
                self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.GREEN)
                self.behavior.color = Color.GREEN
            else:
                self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.RED)
                self.behavior.color = Color.ORANGE
            self.state = self.ColorState.PRESSED

        if self.state == self.ColorState.PRESSED and self.behavior.robot.io.button_state == self.behavior.robot.io.ButtonState.RELEASED:
            self.state = self.ColorState.IDLE

        if self.behavior.robot.io.cord_state == self.behavior.robot.io.CordState.OUT:
            return StateInitialWait

    def deinit(self):
        self.behavior.start_match()
        if self.behavior.color == Color.ORANGE:
            self.behavior.robot.locomotion.reposition_robot(0, 100, 0)
        else:
            self.behavior.robot.locomotion.reposition_robot(1800, 100, math.pi)



class StateOrangeCrossing(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.reposition_robot(1130+45, 220.25, 3*math.pi/2)
        p1 = self.behavior.robot.locomotion.Point(1130 + 45, Y_CROSSING)
        p2 = self.behavior.robot.locomotion.Point(1500, Y_CROSSING)
        self.behavior.robot.locomotion.follow_trajectory([p1, p2], math.pi, -75)  # Arriere


    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished:
            return StateOrangeLineRecalage

        # TODO : US
    def deinit(self):
        pass


class StateGreenCrossing(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.reposition_robot(X_SWITCH_GREEN + BUMPER_OFFSET, ROBOT_BACK2FRONT, 3*math.pi/2)
        p1 = self.behavior.robot.locomotion.PointOrient(X_SWITCH_GREEN + BUMPER_OFFSET, Y_CROSSING, 3*math.pi/2)
        p2 = self.behavior.robot.locomotion.PointOrient(X_SWITCH_GREEN + 200, Y_CROSSING, THETA_INIT_GREEN)
        # self.behavior.robot.locomotion.follow_trajectory([p1, p2], math.pi, -75)  # Arriere
        self.behavior.robot.locomotion.go_to_orient_point(p1, -75)
        self.behavior.robot.locomotion.go_to_orient_point(p2, 75)

    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished:
            return StateGreenLineRecalage

        # TODO : US
    def deinit(self):
        pass


class StateOrangeLineRecalage(FSMMatch):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.go_to_orient(2800, 1500, math.pi, 50)

    def test(self):
        grayscale = self.behavior.robot.io.get_rgb_grayscale
        if grayscale >= WHITE_GRAYSCALE:
            self.behavior.robot.locomotion.reposition_robot(2390, 1300, math.pi)
            print("White detected : " + str(grayscale))
            return StateEnd

        # TODO : US

    def deinit(self):
        pass


class StateGreenLineRecalage(FSMMatch):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.go_to_orient(200, Y_CROSSING, math.pi, 50)

    def test(self):
        grayscale = self.behavior.robot.io.get_rgb_grayscale
        if grayscale >= WHITE_GRAYSCALE:
            self.behavior.robot.locomotion.reposition_robot(2390, 1300, math.pi)
            print("White detected : " + str(grayscale))
            return StateEnd

        # TODO : US

    def deinit(self):
        pass


class StateRecalageRecup(FSMMatch):
    def __init__(self, behavior):
        self.behavior = behavior
        # p1 = self.behavior.robot.locomotion.Point(2390, 1500)
        # self.behavior.robot.locomotion.follow_trajectory([p1], 0, -75)  # Arriere
        self.behavior.robot.locomotion.go_to_orient(2390, 1500, 0, -70)
        self.wait_for_repositionning = False
        self.recalage_start_time = 0

    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True
            self.recalage_start_time = time.time()

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended or (self.behavior.robot.locomotion.is_trajectory_finished and time.time() - self.recalage_start_time > AFTER_SEESAW_RECALAGE_MAX_TIME):
            self.behavior.robot.locomotion.reposition_robot(3000, 1500, 0)
            return StateEnd

    def deinit(self):
        pass

class StateEnd(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.stop_robot()
        self.behavior.robot.locomotion.reposition_robot(0, 0, 0)  # To stop recalage if any

    def test(self):
        pass

    def deinit(self):
        pass
