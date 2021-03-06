from ..utils.utils import dictionary_from_list
import pandas as pd
import cv2
import numpy as np
import rospy


class BaseObject(object):
    """
    Base Clade Tree node

    Parameters
    ----------
    name : str
    parent : CladeTreeNode
        Parent of this node in Clade Tree hierarchy
    Attributes
    ----------
    occurrences : pd.DataFrame
        Describes instances of clade by its parameters
    """

    clade_parameters = []

    def __init__(self, name, aspects):
        self.name = name
        self.aspects = dictionary_from_list(aspects)
        self.aspect_occurrences = {}  # dictionary of aspects occurances {'aspect_name': df_of_described_contours}
        self.occurrences = pd.DataFrame()

    # def get_occurrences(self):
    #     # make sure occurrences are up to date
    #     self.descirbe()
    #     return self.occurrences

    # def describe(self):
    #     for aspect in self.aspects:
    #         aspect.describe()
    #         # fill aspect occurences dataframe
    #         aspect.aspect_occurrences = self.__class__.contours_to_dataframe(aspect.contours)

    # returns list of contours found on given aspects mask
    def get_aspect_occurrences(self, aspect_name):
        aspect = self.aspects[aspect_name]
        # apply rules to source images to make a mask of occurences
        aspect.apply_rule()
        # find contours on a mask
        aspect.find_contours()
        # translate contours - get dataframe of contours described by clade parameters
        aspect_occurences = self.parametrize_contours(aspect.contours)
        return aspect_occurences

    # returns dictionary of pd.DataFrames {aspect_name:DataFrame}
    def get_aspects_occurrences(self):
        self.aspect_occurrences = {}
        for aspect_name, aspect in self.aspects.items():
            # describe contours from each Aspect, put theme in dictionary
            self.aspect_occurrences[aspect.name] = self.get_aspect_occurrences(aspect.name)
        return self.aspect_occurrences

    @classmethod
    def describe_contour(cls, contour):
        return []

    @classmethod
    def get_param_names(cls):
        return []

    # returns dataframe of aspect contours parametrization
    @classmethod
    def parametrize_contours(cls, contours_list):
        values = cls.contours_to_list(contours_list)
        df = pd.DataFrame(values, columns=cls.get_param_names())
        return df

    @classmethod
    def contours_to_list(cls, contours_list):
        if contours_list:
            values = [cls.describe_contour(contour) for contour in contours_list]
        else:
            values = []
        return values


class Blob(BaseObject):

    clade_parameters = ['c_x', 'c_y', 'radius']

    def __init__(self, name, aspects):
        super(Blob, self).__init__(name, aspects)

    @classmethod
    def describe_contour(cls, contour):
        parent_value = super(Blob, cls).describe_contour(contour)
        (x, y), radius = cv2.minEnclosingCircle(contour)
        value = parent_value + [int(x), int(y), int(radius)]
        return value

    @classmethod
    def get_param_names(cls):
        parent_value = super(Blob, cls).get_param_names()
        value = parent_value + cls.clade_parameters
        return value


class RotatedRectangle(Blob):

    clade_parameters = ['width', 'height', 'angle']

    def __init__(self, name, aspects):
        super(Blob, self).__init__(name, aspects)

    @classmethod
    def describe_contour(cls, contour):
        parent_value = super(RotatedRectangle, cls).describe_contour(contour)
        rect = cv2.minAreaRect(contour)
        (x, y), (width, height), angle = rect
        value = parent_value + [int(width), int(height), int(angle)]
        return value

    @classmethod
    def get_param_names(cls):
        parent_value = super(RotatedRectangle, cls).get_param_names()
        value = parent_value + cls.clade_parameters
        return value
