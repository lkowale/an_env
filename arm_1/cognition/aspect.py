from ..utils.utils import dictionary_from_list
import pandas as pd
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point


class Aspect:

    def __init__(self, aspect_rule, source):
        self.aspect_rule = aspect_rule
        self.source = source
        self.name = self.source.name + '_' + self.aspect_rule.name
        self.occurrence_mask = []
        # pd.DataFrame of contours found on occurance mask
        self.contours = []
        self.aspect_occurrences = None

    def find_occurrences(self):
        pass

    def apply_rule(self):
        # make actual occurance mask
        self.occurrence_mask = self.aspect_rule.apply_rule(self.source.raw_image)


class CameraAspect(Aspect):

    def __init__(self, aspect_rule, source):
        super(CameraAspect, self).__init__(self, aspect_rule, source)
        topic_name = "/camera_" + self.source.name + "/image/compressed_mouse_left"
        self.subscriber = rospy.Subscriber(topic_name, Point, self.set_mask_colour, queue_size=1)

    def update(self):
        # find contours on occurrence_mask
        cnts = cv2.findContours(self.aspect_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.contours = cnts[1]

    def set_mask_colour(self, ros_data):
        x = ros_data.x
        y = ros_data.y
        plain_frame = self.source.plain_frame
        bgr_colour = plain_frame[y, x]
        self.aspect_rule.set_colour(bgr_colour)


class AspectRule:

    def __init__(self):
        pass


class ColourThresholdRule(AspectRule):

    def __init__(self, name):
        AspectRule.__init__(self, name)
        self.BGR_treshold_colour = np.zeros((3,), dtype=int)
        self.LAB_treshold_colour = np.zeros((3,), dtype=int)
        self.LAB_treshold_lower = np.zeros((3,), dtype=int)
        self.LAB_treshold_upper = np.zeros((3,), dtype=int)
        self.colour_delta = 15

    def apply_rule(self, raw_source_image):
        frame = cv2.GaussianBlur(raw_source_image, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        colour_lower = self.LAB_treshold_colour - self.colour_delta
        colour_lower[0] = 0
        colour_upper = self.LAB_treshold_colour + self.colour_delta
        colour_upper[0] = 255
        frame = cv2.inRange(frame, colour_lower, colour_upper)
        frame = cv2.erode(frame, None, iterations=2)
        frame = cv2.dilate(frame, None, iterations=2)
        return frame

    def set_colour_delta(self, value):
        self.colour_delta = value

    def set_colour(self, BGR_colour):
        self.BGR_treshold_colour = BGR_colour
        self.LAB_treshold_colour = cv2.cvtColor(np.uint8([[BGR_colour]]), cv2.COLOR_BGR2LAB).ravel()
