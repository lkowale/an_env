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
        # list of contours found on occurance mask
        self.contours = []
        self.aspect_occurrences = None

    def find_contours(self):
        pass

    def apply_rule(self):
        # make actual occurrance mask
        self.occurrence_mask = self.aspect_rule.apply_rule(self.source.raw_image)


class CameraAspect(Aspect):

    def __init__(self, aspect_rule, source):
        super().__init__(aspect_rule, source)

    def find_contours(self):
        # find contours on occurrence_mask
        cnts = cv2.findContours(self.occurrence_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.contours = cnts[1]

    def set_pulled_colour(self, bgr_colour):
        self.aspect_rule.set_colour(bgr_colour)

    # remember to call find_contours() first
    def get_contours(self):
        return self.contours


class AspectRule:

    def __init__(self, name):
        self.name = name


class ColourThresholdRule(AspectRule):

    def __init__(self, name):
        super().__init__(name)
        self.BGR_threshold_colour = np.zeros((3,), dtype=int)
        self.LAB_threshold_colour = np.zeros((3,), dtype=int)
        self.LAB_threshold_lower = np.zeros((3,), dtype=int)
        self.LAB_threshold_upper = np.zeros((3,), dtype=int)
        self.colour_delta = 15
        self.init_red()

    def apply_rule(self, raw_source_image):
        frame = cv2.GaussianBlur(raw_source_image, (5, 5), 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        frame = cv2.inRange(frame, self.LAB_threshold_lower, self.LAB_threshold_upper)
        frame = cv2.erode(frame, None, iterations=2)
        frame = cv2.dilate(frame, None, iterations=2)
        return frame

    def set_colour_delta(self, value):
        self.colour_delta = value

    # parameter BGR_colour is a list of int values [b,g,r]
    def set_colour(self, BGR_colour):
        self.BGR_threshold_colour = np.array(BGR_colour)
        self.LAB_threshold_colour = cv2.cvtColor(np.uint8([[BGR_colour]]), cv2.COLOR_BGR2LAB).ravel()
        self.LAB_threshold_lower = self.LAB_threshold_colour - self.colour_delta
        self.LAB_threshold_lower[0] = 0
        self.LAB_threshold_upper = self.LAB_threshold_colour + self.colour_delta
        self.LAB_threshold_upper[0] = 255

    def init_red(self):
        # upper_red, 15, 40, 29, 178, 99, 185, 162
        # self.aspect_rule.colour_delta = data[1]
        # self.aspect_rule.BGR_treshold_colour = np.array([data[2], data[3], data[4]])
        # self.aspect_rule.LAB_treshold_colour = np.array([data[5], data[6], data[7]])
        self.set_colour_delta(15)
        self.set_colour([40, 29, 178])
