import rclpy
from rclpy.node import Node

import optas

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MapperNode(Node):
    def __init__(self):
        super().__init__("float64multiarray_to_jointstate_mapper_node")

        # Declare parameters
        self.declare_parameter("robot_description")
        self.declare_parameter("joint_type", "position")  # position/velocity/effort

        # Setup robot model
        robot_description = str(self.get_parameter("robot_description").value)
        robot_model = optas.RobotModel(urdf_string=robot_description)

        # Initialize set-state method
        joint_type = str(self.get_parameter("joint_type").value)
        self._set_state = getattr(self, f"_set_{joint_type}")

        # Initialize joint state message
        self._msg = JointState()
        self._msg.name = robot_model.actuated_joint_names

        # Setup joint state publisher
        self._pub = self.create_publisher(JointState, "joint_states/target", 10)

        # Start subscriber
        self.create_subscriber(Float64MultiArray, "target", self._callback)

    def _set_position(self, data):
        self._msg.position = data

    def _set_velocity(self, data):
        self._msg.velocity = data

    def _set_effort(self, data):
        self._msg.effort = data

    def _callback(self, msg):
        self._set_state(msg.data)
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MapperNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
