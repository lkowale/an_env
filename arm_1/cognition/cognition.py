import cv2
import numpy as np
from world_model import WorldModel
# Ros libraries
import roslib
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage
from initialization import ParametersServer


class Cognition:

    def __init__(self):
        self.senses = {'vision': ColorStereoVision()}  # dict of Sense()
        self.local_world_model = WorldModel(self.senses['vision'].eyes)

    def get_clade_occurances(self, clade):
        # make sure clade occurences are up to date
        return self.local_world_model.get_clade(clade).get_occurances()

    def clade_infere(self, clade_name):
        # todo make clade objects positions inference
        pass

    def get_clade_aspects_occurrences(self, clade_name):
        return self.local_world_model.get_clade_aspects_occurrences(clade_name)

    # def get_clade_occurences(self, clade_name):
    #     # update clade occurences
    #     self.get_clade_aspects_occurrences(clade_name)
    #     # make inference of occurrences
    #     self.clade_infere(clade_name)
    #     return self.get_clade(clade_name).occurrences


class Sense:

    def __init__(self):
        pass


class ColorStereoVision(Sense):

    def __init__(self):
        cameras = ParametersServer.get_param('/robot/vision/cameras')
        self.eyes = {name: RosSubscriberEye(name, source) for name, source in cameras}


class Eye:

    def __init__(self, name, source):

        self.name = name
        self.source = source
        self.plain_frame = []
        self.fps = 0


class RosSubscriberEye(Eye):

    def __init__(self, name, source):
        Eye.__init__(self, name, source)

        topic_name = '/vision/eye_' + self.name + '/image/compressed'
        self.subscriber = rospy.Subscriber(topic_name, CompressedImage, self.subscriber_callback,  queue_size=1)

    def subscriber_callback(self, ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.plain_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        # timer = cv2.getTickCount()
        # self.plain_frame = self.camera.read()
        # self. fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
