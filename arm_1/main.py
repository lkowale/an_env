from arm_1.robot.robot import Robot
from arm_1.task.manager import TaskManager
import rospy


class ArmRobot:

    def __init__(self):
        self.robot = Robot()
        self.tm = TaskManager(self.robot)


if __name__ == '__main__':
    arm1 = ArmRobot()
