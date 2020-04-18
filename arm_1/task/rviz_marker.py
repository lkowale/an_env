from .task import PeriodicTask
from sklearn.externals import joblib
from keras.models import load_model
import rospy
from visualization_msgs.msg import Marker


class RvizVisualize(PeriodicTask):

    def __init__(self, name, robot, frequency_hz, start_delay):
        super().__init__(name, robot, frequency_hz, start_delay)
        path = '/learn/64_64_softplus_18.04.2020/'
        model_name = 'model_64x64_b4_e1000_p2'
        self.pipeline = joblib.load(path + 'pipeline_' + model_name + '.pkl')
        self.pipeline.named_steps['regresor'].model = load_model(path + model_name + '.h5')
        self.pub = rospy.Publisher('robot/marker_visualize', Marker, queue_size=10)

    def update(self):
        # dataframe describes contours with clade specific parameters
        readings = self.robot.cognition.get_clade_aspects_occurrences("red_ball")
        X = [
            readings['upper_red'].loc[0, 'cx'],
            readings['upper_red'].loc[0, 'cy'],
            readings['side_red'].loc[0, 'cx'],
            readings['side_red'].loc[0, 'cy'],
        ]
        # X = dataset[['red_ball_side_red_cx', 'red_ball_side_red_cy', 'red_ball_upper_red_cx', 'red_ball_upper_red_cy']]
        x, y, z = self.pipeline.predict([X])
        # x, y, z = int(x/10), int(y/10), int(z/10)
        x, y, z = int(x), int(y), int(z)
        self.pub.publish(self.compose_marker(x, y, z))
        # message = "Inf x:{},y:{},z:{}".format(x, y, z)
        # print(message)

        return self.state

    def compose_marker(self, x, y, z):

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
