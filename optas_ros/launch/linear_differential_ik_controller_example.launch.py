import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Load robot description
    xacro_filename = os.path.join(
        get_package_share_directory("lbr_storz_tilt_endoscope_description"),
        'urdf',
        'lbr_storz_tilt_endoscope.urdf.xacro',
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
        package='optas_ros',
        executable='optas_controller_node',
        name='controller',
        parameters=[
            {"robot_description": robot_description},
            {"script": script_filename},
            {"config": config_filename},
            {"class_name": "LinearDifferentialIKController"},
            {"sampling_frequency": 100},
        ]
    )

    # Setup rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    )

    return LaunchDescription([
        controller_node,
        rviz_node,
    ])
