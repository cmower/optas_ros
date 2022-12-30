#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from optas_ros.node import ControllerNode

class JointControllerNode(ControllerNode):

    def __init__(self):
        super().__init__('optas_joint_controller_node')
        self._js_pub = None

    def enable_send(self, req):
        self._js_pub = rospy.Publisher(req.data, JointState, queue_size=10)

    def disable_send(self):
        self._js_pub.unregister()
        self._js_pub = None

    def send_target_state(self, target):

        # Pack joint state command
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self._task.robot.actuated_joint_names
        msg.position = target.toarray().flatten().tolist()

        # Publish message to ROS
        self._js_pub.publish(msg)

def main():
    JointControllerNode().spin()

if __name__ == '__main__':
    main()
