import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    hz = 100
    dt = 1.0 / float(hz)

    def __init__(self):
        super().__init__("zero_joint_state_publisher")

        self.msg = JointState()
        self.msg.name = [f"lbr_joint_{i}" for i in range(7)]
        self.msg.position = [0.0] * 7

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(self.dt, self._timer_callback)

    def _timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ZeroJointStatePublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
