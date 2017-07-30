import robot_ivy
from behavior import Behavior
from pathfinding import ThetaStar


class Slave(Behavior):
    def __init__(self, robot):
        super().__init__(robot)
        self.pathfinder = ThetaStar(self.robot)
        self.robot.locomotion.x = 1500
        self.robot.locomotion.y = 1000
        self.robot.ivy.register_callback(robot_ivy.GO_TO_REGEXP, self.go_to)


    def loop(self):
        return
        self.get_trajectory_to((9,9))

    def go_to(self, agent, *arg):
        print(arg[0])
        x, y = arg[0].split(",")
        traj = self.pathfinder.find_path((self.robot.locomotion.x, self.robot.locomotion.y), (int(float(x)), int(float(y))))
        print(traj)
        loco_traj = []
        for pt in traj:
            loco_traj.append(self.robot.locomotion.Point(int(pt[0]), int(pt[1])))
        self.robot.locomotion.abort_trajectory()
        self.robot.locomotion.follow_trajectory(loco_traj, 0, 100)
        self.robot.ivy.send_trajectory()

    def get_trajectory_to(self, point):
        print(self.pathfinder.find_path((self.robot.locomotion.x, self.robot.locomotion.y), point))