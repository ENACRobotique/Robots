from ivy.std_api import *



IVY_APP_NAME = "AI_Robot"

NEW_OBSTACLE_REGEXP = "New Obstacle "


class Ivy:
    def __init__(self, robot, bus):
        self.robot = robot
        IvyInit(IVY_APP_NAME, IVY_APP_NAME + "online", 0, self.on_new_connexion, lambda agent, event: None)
        IvyStart(bus)

    def on_new_connexion(self, agent, event):
        if agent.agent_name == "Pygargue":
            for obstacle in self.robot.obstacles:
                IvySendMsg(NEW_OBSTACLE_REGEXP + obstacle.ivy_message())