from enum import Enum

import time

import math

from behavior import Behavior

FUNNY_ACTION_TIME = 92  # in seconds
END_MATCH_TIME = 95  # in seconds
INITIAL_WAIT = 30 #in seconds

#2017 specific
SMALL_CRATER_COLLECT_DURATION = 5 # in seconds
SMALL_CRATER_FIRE_DURATION = 8 # in seconds
STANDARD_SEPARATION_US = 20 # in cm
FULL_SPEED_CANNON_TIME = 2  # in seconds
AFTER_SEESAW_RECALAGE_MAX_TIME = 5  # in sec
FIRE1_RECALAGE_MAX_TIME = 5  # in sec

class Color(Enum):
    BLUE = "blue"
    YELLOW = "yellow"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self.funny_action_finished = False
        self.state = StateRepositionningPreMatch(self)

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

class StateRepositionningPreMatch(FSMMatch):
    def __init__(self, behavior):
        self.behavior = behavior
        self.repositionning = False
        self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.WHITE)

    def test(self):
        if not self.repositionning and self.behavior.robot.io.cord_state == self.behavior.robot.io.CordState.IN:
            self.behavior.robot.locomotion.do_recalage()
            self.repositionning = True
            self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.RED)
        if self.repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            return StateColorSelection

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


class StateSeesawYellow(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.wait_for_repositionning = False
        self.recalage_start_time = 0
        self.behavior.robot.locomotion.go_to_orient(2150, 1820, 1.5*math.pi, 100)

    def test(self):
        if self.behavior.robot.io.__class__.get_us_distance_by_postion("front_left") <= STANDARD_SEPARATION_US and not self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.__class__.get_us_distance_by_postion("front_left") > STANDARD_SEPARATION_US and self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True
            self.recalage_start_time = time.time()

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            self.behavior.robot.locomotion.reposition_robot(2150, 1955, 1.5*math.pi)
            return StateTraj1Yellow

        if self.wait_for_repositionning and not self.behavior.robot.locomotion.is_recalage_ended and time.time() - self.recalage_start_time > AFTER_SEESAW_RECALAGE_MAX_TIME:
            self.behavior.robot.locomotion.reposition_robot(2150, 1955, 1.5 * math.pi)
            return StateTraj1Yellow

    def deinit(self):
        pass


class StateSeesawBlue(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.wait_for_repositionning = False
        self.behavior.robot.locomotion.go_to_orient(850, 1820, 1.5 * math.pi, 100)
        self.recalage_start_time = 0

    def test(self):
        if self.behavior.robot.io.__class__.get_us_distance_by_postion("front_right") <= STANDARD_SEPARATION_US and not self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.__class__.get_us_distance_by_postion("front_right") > STANDARD_SEPARATION_US and self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True
            self.recalage_start_time = time.time()

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            self.behavior.robot.locomotion.reposition_robot(850, 1955, 1.5*math.pi)
            return StateTraj1Blue

        if self.wait_for_repositionning and not self.behavior.robot.locomotion.is_recalage_ended and time.time() - self.recalage_start_time > AFTER_SEESAW_RECALAGE_MAX_TIME:
            self.behavior.robot.locomotion.reposition_robot(850, 1955, 1.5 * math.pi)
            return StateTraj1Blue

    def deinit(self):
        pass


class StateTraj1Yellow(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        p1 = self.behavior.robot.locomotion.Point(2150, 1920)
        p2 = self.behavior.robot.locomotion.Point(1950, 1350)
        self.behavior.robot.locomotion.follow_trajectory([p1, p2], 1.5 * math.pi, 100)
        self.behavior.robot.locomotion.go_to_orient(1950, 1450, 1.5* math.pi, -100)

    def test(self):
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished:
            return StateSmallCrater1Yellow

    def deinit(self):
        pass


class StateTraj1Blue(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        p1 = self.behavior.robot.locomotion.Point(850, 1920)
        p2 = self.behavior.robot.locomotion.Point(1050, 1350)
        self.behavior.robot.locomotion.follow_trajectory([p1, p2], 1.5 * math.pi, 100)
        self.behavior.robot.locomotion.go_to_orient(1050, 1500, 1.5* math.pi, -100)

    def test(self):
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished:
            return StateSmallCrater1Blue

    def deinit(self):
        pass


class StateSmallCrater1Yellow(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.behavior.robot.locomotion.go_to_orient(2150, 1450, 0, 50)
        self.behavior.robot.io.start_ball_picker()
        self.ball_picker_start_time = time.time()

    def test(self):
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        collect_time = time.time() - self.ball_picker_start_time
        if self.behavior.robot.locomotion.is_trajectory_finished and collect_time >= SMALL_CRATER_COLLECT_DURATION:
            return StateTrajFirePositionYellow1

    def deinit(self):
        pass


class StateSmallCrater1Blue(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.behavior.robot.locomotion.go_to_orient(850, 1450, math.pi, 50)
        self.behavior.robot.io.start_ball_picker()
        self.ball_picker_start_time = time.time()

    def test(self):
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        collect_time = time.time() - self.ball_picker_start_time
        if self.behavior.robot.locomotion.is_trajectory_finished and collect_time >= SMALL_CRATER_COLLECT_DURATION:
            return StateTrajFirePositionBlue1

    def deinit(self):
        pass


class StateTrajFirePositionYellow1(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.wait_for_repositionning = False
        self.recalage_start_time = 0
        self.behavior.robot.locomotion.go_to_orient(2550, 1350, 1.5*math.pi, 120)
        self.behavior.robot.locomotion.go_to_orient(2780, 1700, 4.41, -100)

    def test(self):
        # Activate front US before recalage
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False
        # Active back us while recalage
        #if self.behavior.robot.io.rear_distance <= STANDARD_SEPARATION_US and not self.stopped and self.wait_for_repositionning:
        #    self.behavior.robot.locomotion.stop_robot()
        #    self.stopped = True
        #if self.behavior.robot.io.rear_distance > STANDARD_SEPARATION_US and self.stopped and self.wait_for_repositionning:
        #    self.behavior.robot.locomotion.restart_robot()
        #    self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True
            self.recalage_start_time = time.time()

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            self.behavior.robot.locomotion.reposition_robot(2750, 1618, 3 * math.pi / 4)
            return StateFire1

        if self.wait_for_repositionning and not self.behavior.robot.locomotion.is_recalage_ended and time.time() - self.recalage_start_time > AFTER_SEESAW_RECALAGE_MAX_TIME:
            self.behavior.robot.locomotion.reposition_robot(2750, 1618, 3 * math.pi / 4)
            return StateFire1

    def deinit(self):
        pass


class StateTrajFirePositionBlue1(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.wait_for_repositionning = False
        self.recalage_start_time = 0
        self.behavior.robot.locomotion.go_to_orient(420, 1350, 1.5*math.pi, 120)
        self.behavior.robot.locomotion.go_to_orient(250, 1700, 4.91, -100)

    def test(self):
        # Activate front US before recalage
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False
        # Active back us while recalage
        #if self.behavior.robot.io.rear_distance <= STANDARD_SEPARATION_US and not self.stopped and self.wait_for_repositionning:
        #    self.behavior.robot.locomotion.stop_robot()
        #    self.stopped = True
        #if self.behavior.robot.io.rear_distance > STANDARD_SEPARATION_US and self.stopped and self.wait_for_repositionning:
        #    self.behavior.robot.locomotion.restart_robot()
        #    self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True
            self.recalage_start_time = time.time()

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            self.behavior.robot.locomotion.reposition_robot(250, 1618, 3 * math.pi / 4)
            return StateFire1

        if self.wait_for_repositionning and not self.behavior.robot.locomotion.is_recalage_ended and time.time() - self.recalage_start_time > AFTER_SEESAW_RECALAGE_MAX_TIME:
            self.behavior.robot.locomotion.reposition_robot(250, 1618, 3 * math.pi / 4)
            return StateFire1

    def deinit(self):
        pass


class StateFire1(FSMMatch):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.io.start_cannon()
        self.run_motor_start = time.time()
        self.fire_start = 0
        self.firing = False

    def test(self):
        if self.behavior.robot.io.cannon_barrier_state == self.behavior.robot.io.CannonBarrierState.CLOSED and time.time() - self.run_motor_start >= FULL_SPEED_CANNON_TIME:
            self.behavior.robot.io.open_cannon_barrier()
            self.fire_start = time.time()
            self.firing = True

        if self.firing and time.time() - self.fire_start >= SMALL_CRATER_FIRE_DURATION:
            return StateEnd

    def deinit(self):
        self.behavior.robot.io.close_cannon_barrier()
        self.behavior.robot.io.stop_cannon()
        self.behavior.robot.io.stop_ball_picker()


class StateFunnyAction(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.stop_robot()
        self.behavior.robot.io.open_rocket_launcher()

    def test(self):
        return StateEnd

    def deinit(self):
        self.behavior.funny_action_finished = True


class StateEnd(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.stop_robot()
        self.behavior.robot.locomotion.reposition_robot(0, 0, 0)  # To stop recalage if any
        self.behavior.robot.io.stop_ball_picker()
        self.behavior.robot.io.stop_cannon()

    def test(self):
        pass

    def deinit(self):
        pass
