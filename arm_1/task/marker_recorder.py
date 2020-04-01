from .task import Task
import rospy
from std_msgs.msg import String


class MarkerRecorder(Task):

    def __init__(self, name, robot):
        super().__init__(name, robot)
        self.states = ['ready', 'next_position', 'marker_record', 'ended']
        self.state = 'ready'
        self.subscribers = [rospy.Subscriber('marker_recorder/start_button', String, self.start_button),
                            rospy.Subscriber('marker_recorder/onplace_button', String, self.onplace_button)]
        self.publisher = rospy.Publisher('marker_recorder/marker_position', String)
        self.positions = MarkerPositions(range(0, 500, 100), range(0, 400, 100), [355, 255, 155, 55])
        self.file_name = 'marker_recorder_data.csv'
        self.readings_data = []
        self.current_position = []
        self.next_position = None

    def start_button(self):
        if self.state == 'ready':
            self.state = 'next_position'
            self.get_next_position()

    def onplace_button(self):

    def get_next_position(self):
        # returns tuple of (x, y, z) values defines position in  R3 robots workbench_frame
        self.current_position = next(self.positions)
        # if there is next position send command to rqt
        if self.current_position:
            self.publisher
        else:
            self.state = 'done'
            self.save_readings()

    def save_readings(self):

class MarkerPositions:

    def __init__(self, x_list, y_list, z_list):

        self.x_list = x_list
        self.y_list = y_list
        self.z_list = z_list

        self.x_iter = iter(x_list)
        self.y_iter = iter(y_list)
        self.z_iter = iter(z_list)

        self.x = None
        self.y = next(self.y_iter, False)
        self.z = next(self.z_iter, False)

    def __iter__(self):
        return self

    def __next__(self):
        self.x = next(self.x_iter, False)
        # if x itertors ended make new one and take next y
        if self.x is False:
            self.x_iter = iter(self.x_list)
            self.x = next(self.x_iter, False)
            self.y = next(self.y_iter, False)
            # if y iterators ended make new one and take next z
            if self.y is False:
                self.y_iter = iter(self.y_list)
                self.y = next(self.y_iter, False)
                self.z = next(self.z_iter, False)
                # experiment is over
                if self.z is False:
                    return False

        return [self.x, self.y, self.z]
