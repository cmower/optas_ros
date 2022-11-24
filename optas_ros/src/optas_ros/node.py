import os
import sys
import rospy
import traceback
import importlib
from optas_ros.srv import Load, LoadResponse
from sensor_msgs.msg import JointState


class Node:


    def __init__(self, node_name):

        # Setup
        rospy.init_node(node_name)
        self._task = None

        # Setup service
        rospy.Service('load', Load, self._srv_load)


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
        """Service that loads a task"""
        try:
            self.load(req.user_script_filename, req.task_cls_name)
            message = f'loaded {req.task_cls_name} from {req.user_script_filename}'
            success = True
        except:
            message = f'failed to load {req.task_cls_name} from {req.user_script_filename}\n{traceback.format_exc()}'
            success = False
        return LoadResponse(success=success, message=message)


    def spin(self):
        rospy.spin()
