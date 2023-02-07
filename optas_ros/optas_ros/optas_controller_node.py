import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import importlib

class ControllerNode(Node):

    def __init__(self):

        super().__init__('optas_controller_node')

        # Parameters
        self.declare_parameter('script')
        self.declare_parameter('class_name', 'MyController')
        self.declare_parameter('sampling_frequency', 50)
        self.declare_parameter('joint_state_type', 'position')  # position/velocity/effort

        script = str(self.get_parameter('script').value)
        cls_name = str(self.get_parameter('class_name').value)
        self._hz = int(self.get_parameter('sampling_frequency').value)
        self._js_type = str(self.get_parameter('joint_state_type').value)

        # Setup controller
        self._controller = None
        self.load_controller(script, cls_name)

        # Setup joint state publisher
        self._joint_state_publisher = self.create_publisher(JointState, 'joint_states/target')

        # Setup service
        self._timer = None
        self.create_service(AddTwoInts, 'optas_controller/toggle', self.toggle_timer)

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
            messaged = 'stopped timer'

        response.success = True
        response.message = message

        return response

    def load_controller(self, script, cls_name):

        if not os.path.exists(script):
            self.get_logger().error(f"Script not found: {script}")
            return

        # Load class handle from given script
        spec = importlib.util.spec_from_file_location('user_module', script)
        module = importlib.util.module_from_spec(spec)
        sys.modules['module.name'] = module
        spec.loader.exec_module(module)
        Controller = getattr(module, cls_name)

        # Setup controller
        self._controller = Controller(self)

    def _timer_calback(self):

        # Return when controller not ready to start
        if not self._controller.is_ready(): return

        # Reset and solve problem
        self._controller.reset()
        self._controller.solve()

        # Get solution and pack/publish joint state message
        msg = JointState(name=self._controller.get_joint_names())
        setattr(msg, self._js_type, self._controller.get_joint_state_solution())
        self._joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
