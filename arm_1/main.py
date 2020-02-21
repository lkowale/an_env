from .robot.robot import Robot
from .task import TaskManager
import rospy


class ArmRobot:

    def __init__(self):
        rospy.init_node("arm_robot", anonymous=False)
        self.robot = Robot()
        self.tm = TaskManager(self.robot)
        # service to show available Aspects

