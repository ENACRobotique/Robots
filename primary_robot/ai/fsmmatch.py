from enum import Enum

import time

import math

from behavior import Behavior

FUNNY_ACTION_TIME = 92  # in seconds
END_MATCH_TIME = 95  # in seconds
INITIAL_WAIT = 35 #in seconds

#2017 specific
SMALL_CRATER_COLLECT_DURATION = 4  # in seconds
SMALL_CRATER_FIRE_DURATION = 4  # in seconds
GREAT_CRATER_COLLECT_DURATION = 1  # in seconds
STANDARD_SEPARATION_US = 20  # in cm
FULL_SPEED_CANNON_TIME = 2  # in seconds
AFTER_SEESAW_RECALAGE_MAX_TIME = 5  # in sec
FIRE1_RECALAGE_MAX_TIME = 3  # in sec
FIRE1_CANNON_POWER = 45  # between 0 and 255
MAX_CANNON_POWER = 105  # between 0 and 255
CANNON_AUGMENTATION_DISTANCE_STEP = 50  # in mm
CANNON_AUGMENTATION_POWER_STEP = 10  # between 0 and 255


class Color(Enum):
    BLUE = "blue"
    YELLOW = "yellow"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self.funny_action_finished = False
        self.state = StateBeginOrange(self)

    def loop(self):
        time_now = time.time()
        if self.start_time is not None and time_now - self.start_time >= FUNNY_ACTION_TIME and not self.funny_action_finished:  # Checks time for funny action!
            if __debug__:
                print("[FSMMatch] Funny Action time")
            next_state = StateFunnyAction
        elif self.start_time is not None and time_now - self.start_time >= END_MATCH_TIME and self.funny_action_finished and self.state.__class__ != StateEnd:
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



class StateBeginOrange(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.reposition_robot(0, 100, 0)
        p1 = self.behavior.robot.locomotion.Point(200, 100)
        p2 = self.behavior.robot.locomotion.Point(1100, 300)
        self.behavior.robot.locomotion.follow_trajectory([p1, p2], 0, 50)
        self.stopped = False

    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished:
            return StateCloseSwitchOrange
            
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
            
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

    def deinit(self):
        pass


class StateCloseSwitchOrange(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.go_to_orient(1100, 0, 3*math.pi/2, 50)

    def test(self):
        if self.behavior.robot.locomotion.is_trajectory_finished:
            return StateEnd

    def deinit(self):
        pass   
    
    
class StateCollection(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior

    def collection(self):
        self.behavior.robot.io.open_trap()
        time.sleep(3)
        self.behavior.robot.io.close_trap()
        time.sleep(2)
        self.behavior.robot.io.sorter_collect_ball_2()
        time.sleep(2)
        self.behavior.robot.io.open_trap()
        time.sleep(3)
        self.behavior.robot.io.sorter_up()
        time.sleep(7)
        self.behavior.robot.io.sorter_collect_ball_1()
        time.sleep(2)
        return StateEnd

    def deinit(self):
        pass

class StateColorSelection(FSMState):
    class ColorState(Enum):
        IDLE = "idle"
        PRESSED = "pressed"

    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.color = Color.BLUE
        self.state = self.ColorState.IDLE
        self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.BLUE)

    def test(self):
        if self.state == self.ColorState.IDLE and self.behavior.robot.io.button_state == self.behavior.robot.io.ButtonState.PRESSED:
            if self.behavior.color == Color.YELLOW:
                self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.BLUE)
                self.behavior.color = Color.BLUE
            else:
                self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.YELLOW)
                self.behavior.color = Color.YELLOW
            self.state = self.ColorState.PRESSED

        if self.state == self.ColorState.PRESSED and self.behavior.robot.io.button_state == self.behavior.robot.io.ButtonState.RELEASED:
            self.state = self.ColorState.IDLE

        if self.behavior.robot.io.cord_state == self.behavior.robot.io.CordState.OUT:
            return StateInitialWait

    def deinit(self):
        self.behavior.start_match()
        if self.behavior.color == Color.YELLOW:
            self.behavior.robot.locomotion.reposition_robot(2955, 1800, math.pi)
        else:
            self.behavior.robot.locomotion.reposition_robot(45, 1800, 0)



class StateInitialWait(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.start_time = time.time()

    def test(self):
        if time.time() - self.start_time > INITIAL_WAIT:
            if self.behavior.color == Color.YELLOW:
                return StateSeesawYellow
            else:
                return StateSeesawBlue

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
