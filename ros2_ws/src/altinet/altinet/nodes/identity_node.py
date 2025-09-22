"""Service node that performs lightweight identity classification."""

from __future__ import annotations

from typing import Optional

from ..utils.identity import (
    IdentityClassificationConfig,
    IdentityClassifier,
    IdentityObservation,
    compute_area_ratio,
)

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS 2 is unavailable
    import rclpy
    from rclpy.node import Node
    from altinet.srv import CheckPersonIdentity
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    CheckPersonIdentity = None


class IdentityNode(Node):  # pragma: no cover - requires ROS runtime
    """ROS 2 node exposing the ``CheckPersonIdentity`` service."""

    def __init__(self) -> None:
        super().__init__("identity_node")
        if CheckPersonIdentity is None:
            raise RuntimeError("CheckPersonIdentity service definition not available")

        self.declare_parameter("user_min_area_ratio", 0.015)
        self.declare_parameter("min_detection_confidence", 0.25)
        self.declare_parameter("user_confidence_bonus", 0.2)
        self.declare_parameter("guest_confidence_scale", 0.6)
        self.declare_parameter("unknown_confidence_scale", 0.4)

        config = IdentityClassificationConfig(
            user_min_area_ratio=float(self.get_parameter("user_min_area_ratio").value),
            min_detection_confidence=float(
                self.get_parameter("min_detection_confidence").value
            ),
            user_confidence_bonus=float(
                self.get_parameter("user_confidence_bonus").value
            ),
            guest_confidence_scale=float(
                self.get_parameter("guest_confidence_scale").value
            ),
            unknown_confidence_scale=float(
                self.get_parameter("unknown_confidence_scale").value
            ),
        )
        self.classifier = IdentityClassifier(config)

        self.service = self.create_service(
            CheckPersonIdentity,
            "/altinet/check_person_identity",
            self._handle_request,
        )
        self.get_logger().info(
            "Identity classification service ready on /altinet/check_person_identity"
        )

    def _handle_request(self, request, response):
        area_ratio = compute_area_ratio(
            request.w,
            request.h,
            int(request.image_width),
            int(request.image_height),
        )
        observation = IdentityObservation(
            room_id=request.room_id,
            frame_id=request.frame_id,
            track_id=int(request.track_id),
            area_ratio=area_ratio,
            detection_confidence=float(request.detection_confidence),
        )
        result = self.classifier.classify(observation)
        response.label = result.label
        response.is_user = result.is_user
        response.confidence = float(result.confidence)
        response.reason = result.reason
        return response


__all__ = ["IdentityNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = IdentityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
