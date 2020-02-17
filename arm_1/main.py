from robot.robot import Robot
from task import *
import rospy


class ArmRobot:

    def __init__(self):
        rospy.init_node("arm_robot", anonymous=False)
        self.robot = Robot()
        self.current_task = None
        # service to show available Aspects

