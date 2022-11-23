import rospy
from optas_ros.node import Node

class PlannerNode(Node):

    def __init__(self):
        super().__init__('optas_planner_node')
