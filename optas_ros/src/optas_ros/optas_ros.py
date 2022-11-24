import abc
import optas
import rospy


class Task(abc.ABC):

    def __init__(self):
        self._solution = None
        self.robot = None
        self.state_listener = None
        self.builder = None
        self.optimization = None
        self.solver = None
        self.config = None

    def set_config(self, config):
        self.config = config

    @abc.abstractmethod
    def setup_robot(self):
        pass

    @abc.abstractmethod
    def setup_state_listener(self):
        pass

    @abc.abstractmethod
    def specify_problem(self):
        pass

    def build_optimization(self):
        self.optimization = self.builder.build()

    @abc.abstractmethod
    def setup_solver(self):
        pass

    @abc.abstractmethod
    def reset_problem(self):
        pass


class Controller(Task):

    @abc.abstractmethod
    def compute_next_state(self):
        pass


class Planner(Task):

    @abc.abstractmethod
    def plan(self):
        pass
