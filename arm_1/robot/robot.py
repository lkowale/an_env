from ..motion.motion import Motion
from ..cognition.cognition import Cognition
from ..initialization import *
import cv2


class Robot:

    def __init__(self):
        # set paramers on ParameterServer, set camera params
        InitializeRobot.init()
        self.cognition = Cognition()
        self.motion = Motion()

    def update(self):
        if self.current_task:
            self.current_task.update(self)



