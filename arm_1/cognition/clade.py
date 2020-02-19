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

    # todo use metaclass to construct clade classes or/and use AspectParameters class
    def __init__(self, name, aspects):
        self.name = name
        self.aspects = dictionary_from_list(aspects)
        self.occurrences = pd.DataFrame()

    def get_occurrences(self):
        # make sure occurrences are up to date
        self.descirbe()
        return self.occurrences

    def describe(self):
        for aspect in self.aspects:
            aspect.describe()
            # fill aspect occurences dataframe
            aspect.aspect_occurrences = self.__class__.contours_to_dataframe(aspect.contours)

    # returns dictionary of pd.DataFrames {aspect_name:DataFrame}
    def get_aspect_occurrences(self):
        # apply rules to source images
        # get list of contours get from a mask
        ret_dict = {}
        for aspect in self.aspects:

            ret_dict[aspect.name] = aspect.aspect_occurrences
        return ret_dict

    def


    @classmethod
    def describe_contour(cls, contour):
        return []

    @classmethod
    def get_parameters(cls):
        return []

    @classmethod
    def contours_to_dataframe(cls, contours_list):
        values = [cls.describe_contour(contour) for contour in contours_list]
        df = pd.DataFrame(values, columns=cls.get_parameters())
        return df

    @classmethod
    def contours_to_list(cls, contours_list):
        values = [cls.describe_contour(contour) for contour in contours_list]
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
    def get_parameters(cls):
        parent_value = super(Blob, cls).get_parameters()
        value = parent_value + ['c_x', 'c_y', 'radius']
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
    def get_parameters(cls):
        parent_value = super(RotatedRectangle, cls).get_parameters()
        value = parent_value + ['width', 'height', 'angle']
        return value