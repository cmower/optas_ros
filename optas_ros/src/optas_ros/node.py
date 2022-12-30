import os
import re
import abc
import sys
import yaml
import rospkg
import rospy
import traceback
import importlib
from optas_ros.srv import Load, LoadResponse
from sensor_msgs.msg import JointState
from optas_ros.srv import ToggleController, ToggleControllerRequest, ToggleControllerResponse

def replace_package(path):
    """Returns the absolute path to a file. The path can be given relative to ROS package in the format '{ros_package_name}/path/to/file'."""
    rp = rospkg.RosPack()
    matches = re.findall(r'{.+?}', path)
    if len(matches) > 0:
        match = matches[0]
        package = match[1:-1]
        root = rp.get_path(package)
        path = path.replace(match, root)
    return path


def load_config(path):
    """Load config from file."""
    path_ = replace_package(path)
    with open(path_, 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    return config


class Node(abc.ABC):


    def __init__(self, node_name):

        # Setup
        rospy.init_node(node_name)
        self._task = None

        # Setup service
        rospy.Service('load', Load, self._srv_load)


    def load(self, user_script_filename, task_cls_name, config_filename):
        """Load a task from a given script"""

        if self._task is not None:
            self._task.close()

        # Load task class handle from user script
        spec = importlib.util.spec_from_file_location('user_module', user_script_filename)
        module = importlib.util.module_from_spec(spec)
        sys.modules["module.name"] = module
        spec.loader.exec_module(module)
        Task = getattr(module, task_cls_name)

        # Get config
        config = None
        if config_filename:
            config = load_config(config_filename)

        # Setup task
        self._task = Task(config)
        self._task.setup_robot()
        self._task.setup_state_listener()
        self._task.specify_problem()
        self._task.build_optimization()
        self._task.setup_solver()


    def _srv_load(self, req):
        """Service that loads a task"""
        try:
            self.load(req.user_script_filename, req.task_cls_name, req.config_filename)
            message = f'loaded {req.task_cls_name} from {req.user_script_filename}'
            success = True
        except:
            message = f'failed to load {req.task_cls_name} from {req.user_script_filename}\n{traceback.format_exc()}'
            success = False
        return LoadResponse(success=success, message=message)


    def spin(self):
        rospy.spin()


class ControllerNode(Node):


    def __init__(self, node_name):
        super().__init__(node_name)
        self._timer = None
        rospy.Service('toggle', ToggleController, self._srv_toggle_controller)


    def _controller_running(self):
        return self._timer is not None


    def _srv_toggle_controller(self, req):

        if (req.toggle == ToggleControllerRequest.ON) and (not self._controller_running()):
            self.enable_send(req)
            dur = rospy.Duration(1.0/float(req.sampling_freq))
            self._timer = rospy.Timer(dur, self._update)
            success = True
            message = 'started controller'
        elif (req.toggle == ToggleControllerRequest.OFF) and self._controller_running():
            self.disable_send()
            self._timer.shutdown()
            self._timer = None
            success = True
            message = 'stopped controller'
        else:
            success = False
            if self._controller_running():
                message = 'tried to start controller, but it is already running'
            else:
                message = 'tried to stop controller, but it is not running'

        return ToggleControllerResponse(message=message, success=success)


    @abc.abstractmethod
    def enable_send(self):
        pass


    @abc.abstractmethod
    def disable_send(self):
        pass


    @abc.abstractmethod
    def send_target_state(self, target):
        pass


    def _update(self, event):

        # Ensure states are all recieved from listener
        if not self._task.state_listener.recieved_all(): return

        # Reset and solve problem
        self._task.reset_problem()
        target = self._task.compute_next_state()

        # Send target
        self.send_target_state(target)
