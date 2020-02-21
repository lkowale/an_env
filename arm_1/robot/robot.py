from ..motion.motion import Motion
from ..cognition.cognition import Cognition
from ..initialization import *


class Robot:

    def __init__(self):
        self.cognition = Cognition()
        self.motion = Motion()
        InitializeRobot.init()

#todo rospy.spin must be performed every loop
    def update(self):
        if self.current_task:
            self.current_task.update(self)


