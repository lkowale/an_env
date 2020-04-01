import cv2
import numpy as np

# Ros libraries
import roslib
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage

from ..initialization import ParametersServer
from ..utils.utils import dictionary_from_list

from .aspect import CameraAspect
from .aspect import ColourThresholdRule
from .clade import Blob


class Cognition:

    def __init__(self):
        # init senses
        self.senses = {'vision': ColorStereoVision()}

        red_marker_aspects = [
            CameraAspect(ColourThresholdRule('red'), self.senses['vision'].eyes['upper']),
            CameraAspect(ColourThresholdRule('red'), self.senses['vision'].eyes['side'])
        ]

        clades = [
            Blob("red_ball", red_marker_aspects)
        ]

        # self.aspects = dictionary_from_list(aspects)
        self.clades = dictionary_from_list(clades)

    # get clade occurs absolute positions
    def get_clade_occurances(self, clade):
        # todo position infere
        return self.clades[clade].get_occurances()

    # get aspect view positions
    # returns dictionary of pd.DataFrames {aspect_name:DataFrame}
    def get_clade_aspects_occurrences(self, clade_name):
        return self.clades[clade_name].get_aspects_occurrences()


    # not sure if its necessary - maybe better to update only needed clades?
    # # describe world through clades and its aspects
    # # aspects are updating and populating their aspect_occurrences DataFrame with clade unique parameters
    # def update(self):
    #     for clade in self.clades:
    #         clade.update()


class Sense:

    def __init__(self):
        pass


class ColorStereoVision(Sense):

    def __init__(self):
        cameras = ParametersServer.get_param('cameras')

        self.eyes = {camera: RosSubscriberEye(camera, int(cameras[camera]['source'])) for camera in cameras}


class Eye:

    def __init__(self, name, source):
        self.name = name
        self.source = source
        self.raw_image = []
        self.fps = 0


class RosSubscriberEye(Eye):

    def __init__(self, name, source):
        super().__init__(name, source)

        self.topic_name = '/camera_' + self.name + '/image/compressed'
        self.subscriber = rospy.Subscriber(self.topic_name, CompressedImage, self.subscriber_callback, queue_size=1)

    def subscriber_callback(self, ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

