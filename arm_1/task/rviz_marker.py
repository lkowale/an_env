from .task import PeriodicTask
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from arm_1.model.kerasmodel import KerasModel


class RvizVisualize(PeriodicTask):

    def __init__(self, name, robot, frequency_hz, start_delay):
        super().__init__(name, robot, frequency_hz, start_delay)
        self.model = KerasModel('model_64x64_b4_e2000_p2')
        self.model.load()
        self.pub = rospy.Publisher('robot/marker_visualize', Marker, queue_size=10)

    def update(self):
        if self.is_time_window_exceeded():
            # dataframe describes contours with clade specific parameters
            readings = self.robot.cognition.get_clade_aspects_occurrences("red_ball")
            # check if there are any readings
            if self.arent_empty(readings):
                X = [
                    readings['upper_red'].loc[0, 'c_x'],
                    readings['upper_red'].loc[0, 'c_y'],
                    readings['side_red'].loc[0, 'c_x'],
                    readings['side_red'].loc[0, 'c_y'],
                ]
                # X = dataset[['red_ball_side_red_cx', 'red_ball_side_red_cy', 'red_ball_upper_red_cx', 'red_ball_upper_red_cy']]
                marker_position = self.model.pipeline.predict([X])
            else:
                marker_position = np.zeros((1, 3))
            self.pub.publish(self.compose_marker(marker_position))
            # message = "Inf x:{},y:{},z:{}".format(x, y, z)
            # print(message)

    def arent_empty(self, readings):
        for aspect, a_readings in readings.items():
            if len(a_readings) == 0:
                return False
        return True

    def compose_marker(self, pa):
        x, y, z = pa[0, 0], pa[0, 1], pa[0, 2]
        # x, y, z = int(x/10), int(y/10), int(z/10)
        x, y, z = int(x), int(y), int(z)
        marker = Marker()
        marker.header.frame_id = '/my_frame'
        marker.id = 0
        marker.ns = 'vgr'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.lifetime = rospy.Duration(10)
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        return marker
