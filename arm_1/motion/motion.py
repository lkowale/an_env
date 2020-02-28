from ..utils.utils import *
from .limb_interface import SerialLimbInterface
import threading
from ..initialization import ParametersServer


class Motion:

    def __init__(self):
        arm_joints_params = ParametersServer.get_param('robot/arm')
        port = ParametersServer.get_param('hardware_interface/port')
        speed = ParametersServer.get_param('hardware_interface/speed')
        limb_interface = SerialLimbInterface(port, speed)
        self.arm = Limb('arm', limb_interface, arm_joints_params)


class Limb:

    # time added to total wait for end_of_move time
    move_time_offset = 1000

    def __init__(self, name, hardware_interface, joints_params):
        self.name = name
        self.joints = []
        for joint, params in joints_params.items():
            joint_type = globals()[params['type']]
            self.joints.append(joint_type(joint, params))
        self.hardware_interface = hardware_interface
        self.states = ['ready', 'busy']
        self.state = 'ready'

    def move(self, where):
        # check if move position does not exceed allowed joints positions
        if all([joint.move_allowed(where[joint.name]) for joint in self.joints]):
            for joint, position in where:
                self.joints[joint].previous_position = self.joints[joint].set_position
                self.joints[joint].set_position = position
            self.state = 'busy'
            # set Timer to calculated wait time
            threading.Timer(self.calculate_wait_time(), self.stopped).start()
            self.hardware_interface.send(where)

    def calculate_wait_time(self):
        wait_time_s = (max([joint.calculate_moving_time for joint in self.joints]) + self.move_time_offset)/1000.0
        print("calculate_wait_time :{}".format(wait_time_s))
        return wait_time_s

    def stopped(self):
        self.state = 'ready'


class Joint:

    def __init__(self, name, params):
        self.name = name
        self.params = params
        self.p_min = self.params['p_min']
        self.p_max = self.params['p_max']
        self.set_position = 0
        self.previous_position = 0

    def move_allowed(self, position):
        return self.p_min < position < self.p_max

    def calculate_wait_time(self):
        pass


class DCMotor(Joint):

    def calculate_wait_time(self):
        return self.previous_position - self.set_position


class Servo(Joint):

    def __init__(self, name, params):
        super().__init__(name, params)
        self.move_speed_pdgree = self.params['move_speed_pdgree']

    def calculate_wait_time(self):
        return (self.previous_position - self.set_position) * self.move_speed_pdgree
