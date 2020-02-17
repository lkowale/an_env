from initialization import *


# todo robot should launch service that informs what kind of tasks its performs
# to be performed by robot, so the can be listed and picked by user form tasks list

class Task:

    def __init__(self, name, robot, recurrent=False):
        self.name = name
        self.states = []
        self.robot = robot
        self.chain = []
        self.state = None
        self.recurrent = recurrent

    def update(self):
        pass


class LimbMove(Task):

    def __init__(self, robot, where_to):  # world position where_to
        Task.__init__(robot)
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

    def __init__(self, robot):
        where_to = ParametersServer.get_param("/robot/arm/initial_position")
        LimbMove.__init__(self, robot, where_to)

