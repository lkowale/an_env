from .task import Task
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import pandas as pd


# messasge driven task
class MarkerRecorder(Task):

    def __init__(self, name, robot):
        super().__init__(name, robot)
        self.states = ['ready', 'next_position', 'marker_record', 'ended']
        self.state = 'ready'
        self.subscribers = [rospy.Subscriber('marker_recorder/start_button', String, self.start_button),
                            rospy.Subscriber('marker_recorder/onplace_button', String, self.onplace_button)]
        self.publisher = rospy.Publisher('marker_recorder/marker_position', Point, queue_size=1)
        self.position = MarkerPositions(range(0, 500, 100), range(0, 400, 100), [355, 255, 155, 55])
        self.file_name = 'marker_recorder_data.csv'
        self.readings_data = []
        self.current_position = []
        self.point = Point()

    def start_button(self):
        if self.state == 'ready':
            self.state = 'next_position'
            self.readings_data = pd.DataFrame()
            self.get_next_position()

    def onplace_button(self):
        if self.state == 'next_position':
            self.take_readings()
            self.get_next_position()

    def get_next_position(self):
        # get [x, y, z] values defines position in  R3 robots workbench_frame
        self.current_position = next(self.position)
        # if there is next position send command to rqt
        if self.current_position:
            cp = self.current_position
            p = self.point
            p.x = cp[0]
            p.y = cp[1]
            p.z = cp[2]
            self.publisher.publish(p)
        else:
            self.state = 'done'
            self.save_readings()

    def take_readings(self):
        # take readings of marker aspects, its a dictionary of dataframes,
        # dataframe describes contours with clade specific parameters
        readings = self.robot.cognition.get_clade_aspects_occurrences("red_ball")
        # assume that clade has only and at least one occurance on each aspect
        t_readings = self.translate_readings(readings)
        # append theme to local readings
        self.readings_data = pd.concat(self.readings_data, t_readings)


    def save_readings(self):
        # save local readings to file
        pass

    # prepare data to be saved
    def translate_readings(self, readings):
        # translate dictionary of dataframes to one row dataframe
        clade_occurence = pd.DataFrame()
        for key, aspect_occ in readings.items():
            # get rid of radius attribute
            aspect_occ = aspect_occ.drop('radius', 1)
            # change parameters name by adding aspect name
            # todo
            # for each aspect occurence extract one and add it to dictionary
        return None


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
