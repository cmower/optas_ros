import abc
import optas
import rospy
import tf2_ros
from sensor_msgs.msg import JointState

__all__ = [
    'Listener',
    'TopicListener',
    'JointStateListener',
    'TfListener',
    'StateListener',
]


class Listener:

    def __init__(self):
        self._msg = None

    def recieved(self):
        return self._msg is not None

    def get(self):
        return self._msg

    def close(self):
        pass


class TopicListener(Listener):

    def __init__(self, topic_name, topic_type):
        super().__init__()
        self._topic_name = topic_name
        self._topic_type = topic_type
        self._sub = rospy.Subscriber(topic_name, topic_type, self._callback)

    def _callback(self, msg):
        self._msg = msg

    def close(self):
        self._sub.unregister()


class JointStateListener(TopicListener):

    def __init__(self, optas_robot_model, topic_name='joint_states'):
        self._optas_robot_model = optas_robot_model
        super().__init__(topic_name, JointState)

    def get(self):
        q = optas.DM.zeros(self._optas_robot_model.ndof)
        for i, name in enumerate(self._optas_robot_model.actuated_joint_names):
            idx = self._msg.name.index(name)
            q[i] = self._msg.position[idx]
        return q


class TfListener(Listener):

    def __init__(self, parent, child, hz=50):
        super().__init__()
        self.parent = parent
        self.child = child
        self._buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._buf)
        dur = rospy.Duration(1.0/float(hz))
        self._timer = rospy.Timer(dur, self._update)

    def _update(self, event):
        try:
            self._msg = self._buf.lookup_transform(self.parent, self.child, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def close(self):
        self._timer.shutdown()

class StateListener(abc.ABC):

    def __init__(self, states={}):
        self.states = states

    def recieved_all(self):
        return all(s.recieved() for s in self.states.values())

    def get_state(self, label):
        return self.states[label].get()

    def close(self):
        for state in self.states.values():
            state.close()
