from enum import Enum

import time

import math

from behavior import Behavior

FUNNY_ACTION_TIME = 90  # in seconds
END_MATCH_TIME = 95  # in seconds

#2017 specific
SMALL_CRATER_COLLECT_DURATION = 8 # in seconds
SMALL_CRATER_FIRE_DURATION = 7 # in seconds
STANDARD_SEPARATION_US = 20 # in cm
FULL_SPEED_CANNON_TIME = 2  # in seconds

class Color(Enum):
    BLUE = "blue"
    YELLOW = "yellow"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self.state = StateColorSelection(self)

    def loop(self):
        time_now = time.time()
        if self.start_time is not None and time_now - self.start_time >= FUNNY_ACTION_TIME:  # Checks time for funny action!
            next_state = StateFunnyAction
        elif self.start_time is not None and time_now - self.start_time >= END_MATCH_TIME:
            next_state = StateEnd
        else:
            next_state = self.state.test()
        if next_state is not None:
            self.state.deinit()
            self.state = next_state(self)

    def start_match(self):
        self.start_time = time.time()



class FSMState:
    def __init__(self, behavior):
        raise NotImplementedError("this state is not defined yet")

    def test(self):
        raise NotImplementedError("test of this state is not defined yet !")

    def deinit(self):
        raise NotImplementedError("deinit of this state is not defined yet !")


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
            if self.behavior.color == Color.YELLOW:
                return StateTraj1Yellow
            else:
                return StateTraj1Blue

    def deinit(self):
        self.behavior.start_match()
        if self.behavior.color == Color.YELLOW:
            self.behavior.robot.locomotion.reposition_robot(2955, 1800, math.pi)
        else:
            self.behavior.robot.locomotion.reposition_robot(200, 1800, 0)





class StateTraj1Yellow(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        p1 = self.behavior.robot.locomotion.Point(2150, 1820)
        p2 = self.behavior.robot.locomotion.Point(1850, 1700)
        p3 = self.behavior.robot.locomotion.Point(1980, 1450)
        self.behavior.robot.locomotion.follow_trajectory([p1, p2, p3], theta=0, speed=100)

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
        pass

    def test(self):
        pass

    def deinit(self):
        pass

class StateSmallCrater1Yellow(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.behavior.robot.locomotion.go_to_orient(2250, 1450, 0, 50)
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



class StateTrajFirePositionYellow1(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.stopped = False
        self.wait_for_repositionning = False
        #p1 = self.behavior.robot.locomotion.Point(2700, 1450)
        self.behavior.robot.locomotion.go_to_orient(2700, 1350, 4.71, 120)
        #p2 = self.behavior.robot.locomotion.Point(2700, 1680)
        self.behavior.robot.locomotion.go_to_orient(2850, 1700, 4.41, -100)

    def test(self):
        if self.behavior.robot.io.front_distance <= STANDARD_SEPARATION_US and not self.stopped:
            self.behavior.robot.locomotion.stop_robot()
            self.stopped = True
        if self.behavior.robot.io.front_distance > STANDARD_SEPARATION_US and self.stopped:
            self.behavior.robot.locomotion.restart_robot()
            self.stopped = False

        if self.behavior.robot.locomotion.is_trajectory_finished and not self.wait_for_repositionning:
            self.behavior.robot.locomotion.do_recalage()
            self.wait_for_repositionning = True

        if self.wait_for_repositionning and self.behavior.robot.locomotion.is_recalage_ended:
            self.behavior.robot.locomotion.reposition_robot(2850, 1618)
            return StateFireYellow1

    def deinit(self):
        pass

class StateFireYellow1(FSMMatch):

    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.io.start_cannon()
        self.run_motor_start = time.time()
        self.fire_start = 0
        self.firing = False

    def test(self):
        if self.robot.io.cannon_barrier_state == self.robot.io.CannonBarrierState.CLOSED and time.time() - self.run_motor_start >= FULL_SPEED_CANNON_TIME:
            self.robot.io.open_cannon_barrier()
            self.fire_start = time.time()
            self.firing = True

        if self.firing and time.time() - self.fire_start >= SMALL_CRATER_FIRE_DURATION:
            return StateEnd

    def deinit(self):
        self.behavior.robot.io.close_cannon_barrier()
        self.behavior.robot.io.stop_cannon()



class StateFunnyAction(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.stop_robot()
        self.behavior.robot.io.open_rocket_launcher()

    def test(self):
        return StateEnd

    def deinit(self):
        pass


class StateEnd(FSMState):
    def __init__(self, behavior):
        self.behavior = behavior
        self.behavior.robot.locomotion.stop_robot()

    def test(self):
        pass

    def deinit(self):
        pass
