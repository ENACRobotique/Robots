from ivy.std_api import *



IVY_APP_NAME = "AI_Robot"

NEW_OBSTACLE_REGEXP = "New Obstacle "
GO_TO_REGEXP = "Go to (.*)"
NEW_TRAJECTORY_REGEXP = "New trajectory {}"



class Ivy:
    def __init__(self, robot, bus):
        self.robot = robot
        IvyInit(IVY_APP_NAME, IVY_APP_NAME + "online", 0, self.on_new_connexion, lambda agent, event: None)
        IvyStart(bus)

    def on_new_connexion(self, agent, event):
        if agent.agent_name == "Pygargue":
            for obstacle in self.robot.obstacles:
                IvySendMsg(NEW_OBSTACLE_REGEXP + obstacle.ivy_message())

    def register_callback(self, regexp, callback):
        IvyBindMsg(callback, regexp)

    def send_trajectory(self):
        traj = ""
        for point in self.robot.locomotion.current_trajectory:
            pt = point.point
            traj += str(pt.x) + "," + str(pt.y)
            traj += ";"
        IvySendMsg(NEW_TRAJECTORY_REGEXP.format(traj[:-1]))
