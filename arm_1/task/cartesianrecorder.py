from task.task import Task
import time
import pandas as pd
import rospy


class CartesianRecorderPositions:

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
        # if itertors ended make new one and take next shoulder
        if self.x is False:
            self.x_iter = iter(self.x_list)
            self.x = next(self.x_iter, False)
            self.y = next(self.y_iter, False)
            # if iterators ended make new one and take next torso
            if self.y is False:
                self.y_iter = iter(self.y_list)
                self.y = next(self.y_iter, False)
                self.z = next(self.z_iter, False)
                # experiment is over
                if self.z is False:
                    return False

        return self.x, self.y, self.z


class CartesianRecorder(Task):

    def __init__(self, robot):
        Task.__init__(robot)
        self.states = ['ready', 'new_position', 'wait_confirm', 'save_readings', 'save_to_file', 'done']
        self.state = 'ready'
        self.position_generator = CartesianRecorderPositions(range(0, 500, 100), range(0, 400, 100), [355, 255, 155, 55])
        self.file_name = 'CartesianRecorder_data.csv'
        self.readings_data = []
        self.current_position = None
        self.next_position = None
        self.start_time = time.time()
        self.marker_name = 'red_ball'

    def step_forward(self):

        if self.state == 'ready':
            self.state = 'new_position'

        if self.state == 'new_position':
            self.current_position = next(self.position_generator)
            # there is another valid position - send message to operator to put marker on position
            if self.current_position:
                source = self.__class__.__name__
                cp = self.current_position
                message = "Set {},{},{} and confirm".format(cp[0], cp[1], cp[2])
                self.robot.user_interface.out_message(source, message)
                self.state = 'wait_confirm'
            else:
                self.state = 'save_to_file'

        if self.state == 'wait_confirm':
            destination = self.__class__.__name__
            if self.robot.user_interface.in_message(destination, dispatch=True):
                self.state = 'save_readings'

        if self.state == 'save_readings':
            readings = self.robot.cognition.get_clade_aspects_occurrences(self.marker_name)
            translated_readings = self.translate_readings(readings)
            self.readings_data.append(translated_readings)

        if self.state == 'save_to_file':
            df = pd.DataFrame(self.readings_data)
            with open(self.file_name, 'w') as fd:
                df.to_csv(fd, index=False)
            elapsed_time = time.time() - self.start_time
            print("Elapsed time " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))
            self.state = 'done'

    def translate_readings(self, readings):
        # take followed object posiitons and prepare theme to be saved
        return readings
