import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    # Load robot description
    xacro_filename = os.path.join(
        get_package_share_directory("lbr_storz_tilt_endoscope_description"),
        "urdf",
        "lbr_storz_tilt_endoscope.urdf.xacro",
    )

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_filename])

    # Get paths to script/config
    path_to_optas_ros = get_package_share_directory("optas_ros")
    script_filename = os.path.join(
        path_to_optas_ros,
        "linear_differential_ik_controller.py",
    )
    config_filename = os.path.join(
        path_to_optas_ros,
        "linear_differential_ik_controller_configuration.yaml",
    )

    # Setup controller node
    controller_node = Node(
        package="optas_ros",
        executable="optas_controller_node",
        name="controller",
        parameters=[
            {"robot_description": robot_description},
            {"script": script_filename},
            {"config": config_filename},
            {"class_name": "LinearDifferentialIKController"},
            {"sampling_frequency": 100},
        ],
    )

    # Zero joint state
    zero_joint_state_publisher_node = Node(
        package="optas_ros",
        executable="zero_joint_state_publisher_node",
        name="zero_joint_state_publisher",
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Setup rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
    )

    return LaunchDescription(
        [
            controller_node,
            robot_state_publisher_node,
            rviz_node,
            zero_joint_state_publisher_node,
        ]
    )
