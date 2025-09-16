"""Launch the entire Altinet perception stack."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    room_id = LaunchConfiguration("room_id")
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
                parameters=[{"room_id": room_id, "config": "config/yolo.yaml"}],
            ),
            Node(
                package="altinet",
                executable="tracker_node",
                name="tracker_node",
                parameters=["config/tracker.yaml"],
            ),
            Node(
                package="altinet",
                executable="event_manager_node",
                name="event_manager_node",
                parameters=["config/event_rules.yaml"],
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
                parameters=[{"privacy_config": "config/privacy.yaml"}],
            ),
        ]
    )
