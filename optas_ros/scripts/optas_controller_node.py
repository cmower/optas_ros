#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from optas_ros.node import Node
from optas_ros.srv import ToggleController, ToggleControllerResponse

class ControllerNode(Node):

    def __init__(self):
        super().__init__('optas_controller_node')
        self._timer = None
        self._js_pub = rospy.Publisher('joint_states/command', JointState, queue_size=10)
        rospy.Service('toggle', SetBool, self._srv_toggle_controller)

    def _controller_running(self):
        return self._timer is not None

    def _srv_toggle_controller(self, req):

        if (req.toggle == ToggleController.ON) and (not self._controller_running()):
            dur = rospy.Duration(1.0/float(req.sampling_freq))
            self._timer = rospy.Timer(dur, self._update)
            success = True
            message = 'started controller'
        elif (req.toggle == ToggleController.OFF) and self._controller_running():
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

        return ToggleController(message=message, success=success)


    def _update(self, event):

        # Ensure states are all recieved from listener
        if not self._task.state_listener.recieved_all(): return

        # Reset and solve problem
        self._task.reset_problem()
        target = self._task.compute_next_state()

        # Pack joint state command
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self._task.robot.actuated_joint_names
        msg.position = target.toarray().flatten().tolist()

        # Publish message to ROS
        self._js_pub.publish(msg)

def main():
    ControllerNode().spin()

if __name__ == '__main__':
    main()
