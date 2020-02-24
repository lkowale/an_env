import cv2
from .task import CladeObserver


class TaskManager:

    def __init__(self, robot):
        self.robot = robot
        self.tasks = {'red_ball_observer': CladeObserver('red_ball_observer', self.robot)}

    def main_loop(self):

        while True:
            # get time to count fps later
            timer = cv2.getTickCount()

            for name, task in self.tasks:
                task.update()

            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
