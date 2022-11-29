import os
import sys
import tf2_ros
import rospkg
import rospy
from geometry_msgs.msg import TransformStamped
from ros_pybullet_interface.msg import KeyboardEvent
from optas_ros.srv import Load, LoadRequest
from optas_ros.srv import ToggleController, ToggleControllerRequest

class Node:

    srv_timeout = 5.
    LEFT_KEY = 65295
    RIGHT_KEY = 65296

    dt = 1.0/50.
    vel = 0.1

    def __init__(self):
        rospy.init_node('example_controller_node')
        self.p = self.get_initial_target_position()
        self._update_dim_idx = -1
        self.tf_br = tf2_ros.TransformBroadcaster()

    def get_initial_target_position(self):
        buff = tf2_ros.Buffer()
        tf2_ros.TransformListener(buff)

        tf = None
        while tf is None:
            try:
                tf = buff.lookup_transform('rpbi/world', 'rpbi/kuka_lwr/lwr_arm_7_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

        tr = tf.transform.translation
        return [tr.x, tr.y, tr.z]

    def set_transform(self):
        tr = TransformStamped()
        tr.header.stamp = rospy.Time.now()
        tr.header.frame_id = 'rpbi/world'
        tr.child_frame_id = 'target'
        tr.transform.translation.x = self.p[0]
        tr.transform.translation.y = self.p[1]
        tr.transform.translation.z = self.p[2]
        tr.transform.rotation.w = 1.
        self.tf_br.sendTransform(tr)

    def load_controller(self):

        example = os.path.join(rospkg.RosPack().get_path('optas_ros'), 'example')

        req = LoadRequest(
            user_script_filename=os.path.join(example, 'example_controller.py'),
            task_cls_name='ExampleController',
            config_filename=os.path.join(example, 'example_controller.yaml')
        )

        rospy.wait_for_service('load', timeout=self.srv_timeout)
        try:
            load_controller = rospy.ServiceProxy('load', Load)
            resp = load_controller(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"failed to retrieve service handle:\n{e}")
            return False

        if resp.success:
            rospy.loginfo(resp.message)
        else:
            rospy.logerr(resp.message)
            return False

        return True

    def start_controller(self):

        req = ToggleControllerRequest(
            toggle=ToggleControllerRequest.ON,
            topic='rpbi/kuka_lwr/joint_states/target',
            sampling_freq=500,
        )

        rospy.wait_for_service('toggle')
        try:
            start_controller = rospy.ServiceProxy('toggle', ToggleController)
            resp = start_controller(req)
        except rospy.ServiceException as e:
            rospy.logerr(f'failed to retrieve service handle:\n{e}')
            return False

        if resp.success:
            rospy.loginfo(resp.message)
        else:
            rospy.logerr(resp.message)
            return False

        return True

    def start_teleop(self):
        self._sub = rospy.Subscriber('rpbi/keyboard', KeyboardEvent, self._keyboard_callback)
        self._timer = rospy.Timer(rospy.Duration(self.dt), self._update)

    def _keyboard_callback(self, msg):

        if msg.key == ord('q'):
            self._sub.unregister()
            self._timer.shutdown()
            return

        if msg.key == ord('x') and msg.state_str in {'key_was_triggered', ''}:
            self._update_dim_idx = 0
            rospy.loginfo('Move robot end-effector in x-direction')
            return
        elif msg.key == ord('x') and msg.state_str == 'key_was_released':
            self._update_dim_idx = -1
            rospy.loginfo('Robot motion paused.')
            return
        elif msg.key == ord('y') and msg.state_str in {'key_was_triggered', ''}:
            self._update_dim_idx = 1
            rospy.loginfo('Move robot end-effector in y-direction')
            return
        elif msg.key == ord('y') and msg.state_str == 'key_was_released':
            self._update_dim_idx = -1
            rospy.loginfo('Robot motion paused.')
            return
        elif msg.key == ord('z') and msg.state_str in {'key_was_triggered', ''}:
            self._update_dim_idx = 2
            rospy.loginfo('Move robot end-effector in z-direction')
            return
        elif msg.key == ord('z') and msg.state_str == 'key_was_released':
            self._update_dim_idx = -1
            rospy.loginfo('Robot motion paused.')
            return
        elif msg.key == self.LEFT_KEY and msg.state_str == 'key_is_down' and self._update_dim_idx != -1:
            self.p[self._update_dim_idx] -= self.dt*self.vel
        elif msg.key == self.RIGHT_KEY and msg.state_str == 'key_is_down' and self._update_dim_idx != -1:
            self.p[self._update_dim_idx] += self.dt*self.vel

    def _update(self, event):
        self.set_transform()

    def spin(self):
        rospy.spin()

def main():
    node = Node()
    if not node.load_controller():
        return -1
    if not node.start_controller():
        return -1
    node.start_teleop()
    node.spin()
    return 0



if __name__ == '__main__':
    sys.exit(main())
