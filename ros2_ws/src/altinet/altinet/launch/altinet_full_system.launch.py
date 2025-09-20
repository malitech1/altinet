"""Launch the entire Altinet perception stack."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    room_id = LaunchConfiguration("room_id")
    altinet_share = FindPackageShare("altinet")
    return LaunchDescription(
        [
            DeclareLaunchArgument("room_id", default_value="living_room"),
            Node(
                package="altinet",
                executable="camera_node",
                name="camera_node",
                parameters=[{"room_id": room_id, "source": 0, "fps": 20.0}],
            ),
            Node(
                package="altinet",
                executable="detector_node",
                name="detector_node",
                parameters=[
                    {
                        "room_id": room_id,
                        "config": PathJoinSubstitution(
                            [altinet_share, "config", "yolo.yaml"]
                        ),
                    }
                ],
            ),
            Node(
                package="altinet",
                executable="tracker_node",
                name="tracker_node",
                parameters=[
                    PathJoinSubstitution([altinet_share, "config", "tracker.yaml"])
                ],
            ),
            Node(
                package="altinet",
                executable="visualizer_node",
                name="visualizer_node",
                parameters=[{"room_id": room_id}],
            ),
            Node(
                package="altinet",
                executable="event_manager_node",
                name="event_manager_node",
                parameters=[
                    PathJoinSubstitution([altinet_share, "config", "event_rules.yaml"])
                ],
            ),
            Node(
                package="altinet",
                executable="lighting_control_node",
                name="lighting_control_node",
                parameters=[{"cooldown_s": 30.0, "override_duration_s": 300.0}],
            ),
            Node(
                package="altinet",
                executable="ros2_django_bridge_node",
                name="ros2_django_bridge_node",
                parameters=[
                    {
                        "privacy_config": PathJoinSubstitution(
                            [altinet_share, "config", "privacy.yaml"]
                        )
                    }
                ],
            ),
        ]
    )
