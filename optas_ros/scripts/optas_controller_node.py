import rospy
from optas_ros.node import Node
from std_srvs.srv import SetBool, SetBoolResponse

class ControllerNode(Node):

    def __init__(self):
        super().__init__('optas_controller_node')
        self._timer = None
        self._js_pub = rospy.Publisher('joint_states/command', JointState, queue_size=10)
        rospy.Service('toggle', SetBool, self._srv_toggle_controller)

    def controller_running(self):
        return self._timer is not None

    def _srv_toggle_controller(self, req):
        pass  # todo

    def _update(self, event):
        self._task.reset()
        self._task.compute_next_state()

def main():
    ControllerNode().spin()
        
if __name__ == '__main__':
    main()
