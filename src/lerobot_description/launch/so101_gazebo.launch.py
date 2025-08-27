import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lerobot_description = get_package_share_directory("lerobot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        lerobot_description, "urdf", "so101.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(lerobot_description).parent.resolve())
            ]
        )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r ", os.path.join(lerobot_description, "worlds", "so101_with_objects.world")]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "so101"],
    )

    # Bridge for clock
    gz_ros2_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    # Bridge for RGB camera
    gz_ros2_bridge_rgb = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/so101/camera_link/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/so101/camera_link/rgb_camera@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
    )

    # Bridge for depth camera
    gz_ros2_bridge_depth = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/so101/camera_link/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/so101/camera_link/depth_camera@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge_clock,
        gz_ros2_bridge_rgb,
        gz_ros2_bridge_depth
    ])