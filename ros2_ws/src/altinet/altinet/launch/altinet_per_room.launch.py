"""Launch file for a single room pipeline."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from altinet.utils.config import default_yolo_config_path


def generate_launch_description() -> LaunchDescription:
    room_id = LaunchConfiguration("room_id")
    default_config = str(default_yolo_config_path())
    return LaunchDescription(
        [
            DeclareLaunchArgument("room_id", default_value="living_room"),
            DeclareLaunchArgument("camera_config", default_value="config/cameras.yaml"),
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
                parameters=[{"room_id": room_id, "config": default_config}],
            ),
            Node(
                package="altinet",
                executable="tracker_node",
                name="tracker_node",
                parameters=["config/tracker.yaml"],
            ),
            Node(
                package="altinet",
                executable="visualizer_node",
                name="visualizer_node",
                parameters=[{"room_id": room_id}],
            ),
            Node(
                package="altinet",
                executable="identity_node",
                name="identity_node",
            ),
        ]
    )
