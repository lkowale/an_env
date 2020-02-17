from task.task import Task
import time
import pandas as pd
import rospy
from dynamic_reconfigure.server import Server
from ros_dyn_cfg import TresholdColorPickerConfig

#todo there's need to be an adapter that takes aspect name and data to be applied to this adapter
#todo rqt plugin be written


class TresholdColorPicker(Task):

    def __init__(self, robot):
        Task.__init__(robot)
        self.states = ['ready', 'set_colour', 'done']
        self.state = 'ready'
        def
        self.dyn_conf_serv = Server(TresholdColorPickerConfig, TresholdColorPicker.callback)

    def update(self):

        if self.state == 'ready':
            # do nothing - wait further

        if self.state == 'set_colour':

    @staticmethod
    def callback(config, level):
            aspect = self.robot.world.get_aspect()
            return config
