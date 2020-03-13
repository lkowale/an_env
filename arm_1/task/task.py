from ..initialization import *


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


class PeriodicTask(Task):

    def __init__(self, name, robot, recurrent=False, frequency=20):
        Task.__init__(name, robot, recurrent)
        self.frequency = frequency
    # todo in periodical task nedd to check if ferequency of call is fulfilled


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
        if self.ready:
            # todo describe path of clade occurences acquisition
            self.robot.cognition.get_clade_aspects_occurrences("red_ball")
