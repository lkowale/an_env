from ..initialization import *
from time import time
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


# todo robot should launch service that informs what kind of tasks its performs
# to be performed by robot, so the can be listed and picked by user form tasks list
class Task:

    def __init__(self, name, robot):
        self.name = name
        self.states = []
        self.robot = robot
        self.chain = []
        self.state = None

    def update(self):
        pass


class PeriodicTask(Task):

    def __init__(self, name, robot, frequency_hz=20, start_delay=0):
        super().__init__(name, robot)
        self.frequency = frequency_hz
        self.time_window = 1 / self.frequency
        # delay start by 1 sec
        self.start_time = time() + start_delay

    def is_time_window_exceeded(self):
        now = time()
        if now - self.start_time > self.time_window:
            return True
        else:
            return False


class LimbMove(Task):

    def __init__(self, name, robot, where_to):  # world position where_to
        Task.__init__(self, name, robot)
        self.where_to = where_to
        self.states = ['ready', 'busy', 'ended']
        self.state = 'ready'

    def update(self):
        if self.state == 'ready':
            self.robot.motion.arm.move(self.where_to)
            self.state == 'busy'

        if self.state == 'busy':
            if self.robot.motion.arm.state == 'ready':
                self.state = 'ended'

        if self.state == 'ended':
            pass


class MoveToInitialPosition(LimbMove):

    def __init__(self, name, robot):
        where_to = ParametersServer.get_param("/robot/arm/initial_position")
        LimbMove.__init__(self, name, robot, where_to)


class CladeObserver(PeriodicTask):

    def update(self):
        if self.is_time_window_exceeded():
            self.robot.cognition.get_clade_aspects_occurrences("red_ball")


class AspectPublisher:

    def __init__(self, aspect):
        self.aspect = aspect
        topic_name = self.aspect.name + '_mask/image/compressed'
        self.publisher = rospy.Publisher(topic_name, CompressedImage, queue_size=1)

    def publish(self):
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.aspect.occurrence_mask)[1]).tostring()
        # Publish new image
        self.publisher.publish(msg)


class MaskPublisher(PeriodicTask):

    def __init__(self, name, robot, frequency_hz, start_delay):
        super().__init__(name, robot, frequency_hz, start_delay)
        self.publishers = []
        # get marker Clade
        marker = self.robot.cognition.clades['red_ball']
        # for each aspect of red marker publish its mask as an image
        for aspect_name, aspect in marker.aspects.items():
            # create aspect publisher and add it to publishers list
            self.publishers.append(AspectPublisher(aspect))

    def update(self):
        if self.is_time_window_exceeded():
            # publish masks
            for publisher in self.publishers:
                publisher.publish()
