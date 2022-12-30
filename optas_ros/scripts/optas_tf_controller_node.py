#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from optas_ros.node import ControllerNode


class TfControllerNode(ControllerNode):


    def __init__(self):
        super().__init__('optas_tf_controller_node')
        self._parent_frame_id = rospy.get_param('parent_frame_id')
        self._child_frame_id = rospy.get_param('child_frame_id')
        self._tf_br = None


    def enable_send(self):
        self._tf_br = tf2_ros.TransformBroadcaster()


    def disable_send(self):
        self._tf_br.pub_tf.unregister()
        self._tf_br = None


    def send_target_state(self, target):

        # Parse target to Python list
        target = target.toarray().flatten().tolist()

        # Extract target position/quaternion
        if len(target) == 3:
            p = target
            q = None
        elif len(target) == 4:
            p = None
            q = target
        elif len(target) == 6:
            p = target[:3]
            q = tf_conversions.transformations.quaternion_from_euler(*target[3:])
        elif len(target) == 7:
            p = target[:3]
            q = target[3:]

        # Pack Tf message
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = self._parent_frame_id
        tf.child_frame_id = self._child_frame_id
        if p is not None:
            tf.transform.translation.x = p[0]
            tf.transform.translation.y = p[1]
            tf.transform.translation.z = p[2]
        if q is not None:
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]

        self._tf_br.sendTransform(tf)


def main():
    TfControllerNode().spin()


if __name__ == '__main__':
    main()
