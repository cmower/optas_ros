import os
import sys
import abc
import rospy
import importlib
from sensor_msgs.msg import JointState


class Node(abc.ABC):


    def __init__(self, node_name):

        # Setup
        rospy.init_node(node_name)
        self._task = None

        # Setup service
        rospy.Service('load', LoadTask, self._srv_load)


    def load(self, user_script_filename, task_cls_name):
        """Load a task from a given script"""

        # Load task class handle from user script
        spec = importlib.util.spec_from_file_location('user_module', user_script_filename)
        module = importlib.util.module_from_spec(spec)
        sys.modules["module.name"] = module
        spec.loader.exec_module(module)
        Task = getattr(module, task_cls_name)

        # Setup task
        self._task = Task()
        self._task.specify_problem()
        self._task.specify_solver()


    def _srv_load(self, req):
        """Service that re-loads a task"""
        self.load(req.user_script_filename, req.task_cls_name)


    def spin(self):
        rospy.spin()
