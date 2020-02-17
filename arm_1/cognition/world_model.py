from utils.utils import dictionary_from_list
import pandas as pd
import cv2
import numpy as np
import rospy
from aspect import CameraAspect, ColourTresholdRule
from clade import Blob


class WorldModel:

    def __init__(self, eyes):
        red_marker_aspects = [
            CameraAspect(ColourTresholdRule('red'), eyes[0]),
            CameraAspect(ColourTresholdRule('red'), eyes[1])
                             ]
        self.clades = {
            Blob("red_ball", red_marker_aspects)
        }

    def get_clade(self, clade_name):
        return self.clades[clade_name]

    def update_all_clades(self):
        for clade in self.clades:
            # update occurences dataframe
            clade.describe()

    def get_clade_aspects_occurrences(self, clade_name):
        clade = self.get_clade(clade_name)
        # update occurences dataframe
        clade.describe()
        return clade.get_aspects_occurrences()

