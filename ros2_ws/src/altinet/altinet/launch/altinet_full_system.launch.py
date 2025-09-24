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
                        "min_detection_interval": 1.5,
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
                executable="face_capture_node",
                name="face_capture_node",
                parameters=[
                    {
                        "room_id": room_id,
                        "capture_cadence_s": 2.0,
                        "minimum_quality": 0.65,
                        "improvement_margin": 0.05,
                        "frame_history": 60,
                        "frame_tolerance_s": 0.2,
                        "face_padding": 0.2,
                        "model_name": "buffalo_l",
                        "model_root": PathJoinSubstitution(
                            [altinet_share, "assets", "models"]
                        ),
                        "det_size": [640, 640],
                        "ctx_id": 0,
                    }
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
            Node(
                package="altinet",
                executable="identity_node",
                name="identity_node",
            ),
        ]
    )
