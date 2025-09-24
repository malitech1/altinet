"""ROS 2 node that accepts face embeddings and updates the gallery."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..services.face_recognition import EnrollmentResult, FaceRecognitionService

_ROS_IMPORT_ERROR: Optional[Exception] = None
try:  # pragma: no cover - optional when ROS 2 is unavailable
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from altinet.srv import FaceEnrolment
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    String = None  # type: ignore
    FaceEnrolment = None  # type: ignore


class FaceEnrollerNode(Node):  # pragma: no cover - requires ROS runtime
    """Expose `/altinet/face_enroller` for gallery updates."""

    def __init__(self) -> None:
        super().__init__("face_enroller")
        self.declare_parameter("gallery_dir", str(Path("assets/face_gallery")))
        gallery_dir = Path(self.get_parameter("gallery_dir").value)
        self.gallery = FaceRecognitionService(gallery_dir)
        self.gallery.register_listener(self._on_enrollment)

        self.publisher = None
        if String is not None:
            self.publisher = self.create_publisher(
                String, "/altinet/face_gallery_updates", 10
            )

        if FaceEnrolment is None:
            raise RuntimeError("FaceEnrolment service definition not available")
        self.service = self.create_service(
            FaceEnrolment, "/altinet/face_enroller", self._handle_request
        )
        self.get_logger().info("Face enroller ready on /altinet/face_enroller")

    # ------------------------------------------------------------------
    # ROS handlers
    # ------------------------------------------------------------------
    def _handle_request(self, request, response):  # pragma: no cover - ROS runtime
        identity = _first_of(request, ["identity", "identity_id"])
        camera_id = _first_of(request, ["camera_id", "camera"])
        track_id = _safe_int(_first_of(request, ["track_id"]))
        quality = float(getattr(request, "quality", 1.0) or 1.0)
        metadata = _parse_metadata(getattr(request, "metadata_json", ""))

        vectors = _extract_vectors(request)
        if not vectors and getattr(request, "vector", None) is not None:
            vectors = [(list(request.vector), metadata)]
        if not vectors and getattr(request, "embedding", None) is not None:
            vectors = [(list(request.embedding), metadata)]
        if not vectors:
            return self._populate_response(response, None)

        enriched = []
        for vector, meta in vectors:
            item_meta = {**metadata, **meta}
            enriched.append((vector, item_meta))

        result = self.gallery.enrol_embeddings(
            identity,
            enriched,
            track_id=track_id,
            camera_id=camera_id,
            quality_default=quality,
        )
        return self._populate_response(response, result)

    def _on_enrollment(self, result: EnrollmentResult) -> None:
        if self.publisher is None:
            return
        payload = {
            "identity": result.identity_id,
            "accepted": [entry.embedding_id for entry in result.accepted],
            "rejected": [entry.metadata.get("reason") for entry in result.rejected],
        }
        message = String()
        message.data = json.dumps(payload)
        self.publisher.publish(message)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _populate_response(response, result: Optional[EnrollmentResult]):
        if response is None:
            return None
        if result is None:
            if hasattr(response, "ok"):
                response.ok = False
            return response
        if hasattr(response, "accepted"):
            response.accepted = result.accepted_count
        if hasattr(response, "rejected"):
            response.rejected = result.rejected_count
        if hasattr(response, "embedding_ids"):
            response.embedding_ids = [entry.embedding_id for entry in result.accepted]
        if hasattr(response, "ok"):
            response.ok = result.accepted_count > 0
        return response


def _parse_metadata(raw: str) -> Dict[str, Any]:
    if not raw:
        return {}
    try:
        data = json.loads(raw)
        if isinstance(data, dict):
            return data
    except ValueError:  # pragma: no cover - defensive
        return {}
    return {}


def _first_of(obj: Any, attributes: Sequence[str]) -> Any:
    for attr in attributes:
        if hasattr(obj, attr):
            value = getattr(obj, attr)
            if value not in (None, ""):
                return value
    return None


def _safe_int(value: Any) -> Optional[int]:
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _extract_vectors(request: Any) -> List[Tuple[Sequence[float], Dict[str, Any]]]:
    vectors: List[Tuple[Sequence[float], Dict[str, Any]]] = []
    if hasattr(request, "embeddings"):
        for item in getattr(request, "embeddings") or []:
            vector = _first_of(item, ["vector", "values"])
            meta = _parse_metadata(getattr(item, "metadata_json", ""))
            if vector is not None:
                vectors.append((list(vector), meta))
    return vectors


__all__ = ["FaceEnrollerNode"]


def main(args=None):  # pragma: no cover - requires ROS runtime
    if rclpy is None:
        message = "ROS 2 dependencies could not be imported"
        if _ROS_IMPORT_ERROR is not None:
            message += f": {_ROS_IMPORT_ERROR}"
        raise RuntimeError(message) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = FaceEnrollerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
