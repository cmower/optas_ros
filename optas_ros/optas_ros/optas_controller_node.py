import os
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import importlib

class ControllerNode(Node):

    def __init__(self):

        super().__init__('optas_controller_node')

        # Parameters
        self.declare_parameter('robot_description')
        self.declare_parameter('script')
        self.declare_parameter('config')
        self.declare_parameter('class_name', 'MyController')
        self.declare_parameter('sampling_frequency', 50)

        robot_description = str(self.get_parameter('robot_description').value)
        script = str(self.get_parameter('script').value)
        config = str(self.get_parameter('config').value)
        cls_name = str(self.get_parameter('class_name').value)
        self._hz = int(self.get_parameter('sampling_frequency').value)

        # Setup controller
        self._controller = self._load_controller(script, cls_name, config, self._hz, robot_description)

        # Setup service
        self._timer = None
        self.create_service(Trigger, 'optas_controller/toggle', self.toggle_timer)

    def toggle_timer(self, request, response):

        if self._timer is None:

            # Start timer
            dt = 1./float(self._hz)
            self._timer = self.create_timer(dt, self._timer_calback)
            messaged = 'started timer'

        else:

            # Stop timer
            self.destroy_timer(self._timer)
            self._timer = None
            self._controller.reset_manager()
            messaged = 'stopped timer'

        response.success = True
        response.message = message

        return response

    def _load_controller(self, script, cls_name, config_filename, hz, robot_description):

        if not os.path.exists(script):
            raise ValueError(f"Script not found: {script}")

        # Load class handle from given script
        spec = importlib.util.spec_from_file_location('user_module', script)
        module = importlib.util.module_from_spec(spec)
        sys.modules['module.name'] = module
        spec.loader.exec_module(module)
        Controller = getattr(module, cls_name)

        # Setup controller
        return Controller(self, 2, config_filename, hz, robot_description)

    def _timer_calback(self):
        if not self._controller.is_ready(): return
        self._controller()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
