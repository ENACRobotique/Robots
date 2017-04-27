from behavior import Behavior


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.state = StateInit(robot)
        self.robot = robot

    def loop(self):
        next_state = self.state.test()
        if next_state is not None:
            self.state.deinit()
            self.state = next_state(self.robot)


class FSMState:
    def __init__(self, robot):
        raise NotImplementedError("this state is not defined yet")

    def test(self):
        raise NotImplementedError("test of this state is not defined yet !")

    def deinit(self):
        raise NotImplementedError("deinit of this state is not defined yet !")


class StateInit(FSMState):
    def __init__(self, robot):
        pass

    def test(self):
        pass

    def deinit(self):
        pass
