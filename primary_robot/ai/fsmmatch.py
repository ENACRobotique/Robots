class FSMMatch:
    def __init__(self, robot):
        self.state = StateInit()
        self.robot = robot

    def loop(self):
        next_state = self.state.test()
        if next_state is not None:
            self.state.deinit()
            self.state = next_state()


class FSMState:
    def __init__(self):
        raise NotImplementedError("this state is not defined yet")

    def test(self):
        raise NotImplementedError("test of this state is not defined yet !")

    def deinit(self):
        raise NotImplementedError("deinit of this state is not defined yet !")


class StateInit(FSMState):
    pass