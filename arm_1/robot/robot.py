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


#todo rospy.spin must be performed every loop
    def update(self):
        if self.current_task:
            self.current_task.update(self)

# todo perform base taks - image subscirbers and aspects masks publishers
def main_loop():

    while True:
        # get time to count fps later
        timer = cv2.getTickCount()

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
