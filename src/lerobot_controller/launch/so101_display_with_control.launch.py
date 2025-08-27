import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")

    # Get the robot description
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(lerobot_description_dir, "urdf", "so101.urdf.xacro"),
        ]),
        value_type=str,
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Joint state publisher that listens to our commands
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{
            "source_list": ["/joint_commands"],  # Listen to our joint commands
            "publish_default_positions": False,   # Don't publish default positions
            "publish_default_velocities": False,  # Don't publish default velocities
            "publish_default_efforts": False      # Don't publish default efforts
        }]
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(lerobot_description_dir, "rviz", "display.rviz")],
    )

    # Our joint controller node
    joint_controller_node = Node(
        package="lerobot_controller",
        executable="joint_controller.py",
        name="joint_controller",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        joint_controller_node,
    ])
