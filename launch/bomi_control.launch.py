from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from reachy_BoMI_ROS2.scenarios import SCENARIO_NAMES, resolve_world_for_scenario


def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    start_rviz = LaunchConfiguration("start_rviz")

    world = resolve_world_for_scenario(scenario)

    reachy_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("reachy_bringup"), "/launch/reachy.launch.py"]
        ),
        launch_arguments={
            "gazebo": "true",
            "start_rviz": start_rviz,
            "start_sdk_server": "false",
            "foxglove": "false",
            "orbbec": "false",
            "world": world,
        }.items(),
    )

    server_socket_node = Node(
        package="reachy_BoMI_ROS2",
        executable="server_socket",
        output="screen",
    )

    cmd_vel_publisher_node = Node(
        package="reachy_BoMI_ROS2",
        executable="cmd_vel_publisher",
        output="screen",
    )

    return [
        reachy_sim,
        server_socket_node,
        cmd_vel_publisher_node,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="familiarization",
                description="Scenario to launch",
                choices=list(SCENARIO_NAMES),
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Whether to start RViz",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ],
    )