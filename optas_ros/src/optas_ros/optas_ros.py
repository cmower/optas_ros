import abc
import optas
import rospy


class Task(abc.ABC):

    def __init__(self):
        self.robot = None
        self.state_listener = None
        self.config = rospy.get_param('~config', {})

    @abc.abstractmethod
    def specify_problem(self):
        pass

    @abc.abstractmethod
    def setup_solver(self):
        pass

    @abc.abstractmethod
    def reset(self):
        pass


class Controller(Task):

    @abc.abstractmethod
    def compute_next_state(self):
        pass


class Planner(Task):

    @abc.abstractmethod
    def plan(self):
        pass
