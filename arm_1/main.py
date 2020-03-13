from .robot.robot import Robot
from .task.manager import TaskManager


class ArmRobot:

    def __init__(self):
        self.robot = Robot()
        self.tm = TaskManager(self.robot)
        self.tm.main_loop()


if __name__ == '__main__':
    arm1 = ArmRobot()
