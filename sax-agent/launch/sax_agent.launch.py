from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    parameters = PathJoinSubstitution([FindPackageShare("sax_agent"), "config", "sax_agent.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="robot_01"),
            DeclareLaunchArgument("bridge_url", default_value="ws://localhost:8000"),
            DeclareLaunchArgument("bridge_token", default_value=""),
            Node(
                package="sax_agent",
                executable="agent_node",
                name="sax_agent",
                output="screen",
                parameters=[
                    parameters,
                    {
                        "robot_id": LaunchConfiguration("robot_id"),
                        "bridge_url": LaunchConfiguration("bridge_url"),
                        "bridge_token": LaunchConfiguration("bridge_token"),
                    },
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
