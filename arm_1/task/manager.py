import cv2
from .task import CladeObserver
from .task import MaskPublisher
from .task import AspectColourPicker
from .marker_recorder import MarkerRecorder
from .rviz_marker import RvizVisualize
import rospy


class TaskManager:

    def __init__(self, robot):
        self.robot = robot
        self.synchronous_tasks = {'red_ball_observer': CladeObserver('red_ball_observer', self.robot, frequency_hz=20, start_delay=1),
                      'mask_publisher': MaskPublisher('mask_publisher', self.robot, frequency_hz=20, start_delay=2)
                                  # ,'marker_recorder': MarkerRecorder('marker_recorder', robot)
                                  ,'marker_recorder': RvizVisualize('RvizVisualize', robot)}
        self.message_driven_tasks = {'asp_colpicker': AspectColourPicker('asp_colpicker', robot)

                                     }

    def main_loop(self):
        rospy.init_node('robot')
        while True:
            # get time to count fps later
            timer = cv2.getTickCount()

            for name, task in self.synchronous_tasks.items():
                task.update()
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
