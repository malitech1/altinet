"""Service node that performs lightweight identity classification."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from ..utils.face_identity import (
    FaceIdentityConfig,
    FaceIdentityResolver,
    FaceSnapshot,
    FaceSnapshotCache,
    load_face_index,
)
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
    try:  # pragma: no cover - optional dependency
        from altinet_interfaces.msg import FaceSnapshot as FaceSnapshotMsg
    except ImportError:  # pragma: no cover - executed when interfaces unavailable
        FaceSnapshotMsg = None  # type: ignore
    try:  # pragma: no cover - optional dependency
        from altinet_interfaces.srv import GetFaceSnapshot
    except ImportError:  # pragma: no cover - executed when interfaces unavailable
        GetFaceSnapshot = None  # type: ignore
except ImportError as exc:  # pragma: no cover - executed during tests
    _ROS_IMPORT_ERROR = exc
    rclpy = None
    Node = object  # type: ignore
    CheckPersonIdentity = None
    FaceSnapshotMsg = None  # type: ignore
    GetFaceSnapshot = None  # type: ignore


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
        self.declare_parameter("face_index_path", "")
        self.declare_parameter("face_index_metadata_path", "")
        self.declare_parameter("face_snapshot_topic", "/altinet/face_snapshot")
        self.declare_parameter("face_snapshot_service", "/altinet/get_face_snapshot")
        self.declare_parameter("snapshot_cache_ttl_sec", 5.0)
        self.declare_parameter("user_similarity_threshold", 0.75)
        self.declare_parameter("guest_similarity_threshold", 0.6)
        self.declare_parameter("unknown_label", "unknown")
        self.declare_parameter("fallback_to_heuristic_for_unknown", False)
        self.declare_parameter("face_snapshot_timeout", 0.5)

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

        cache_ttl = float(self.get_parameter("snapshot_cache_ttl_sec").value)
        self._snapshot_cache = FaceSnapshotCache(cache_ttl)
        face_index_path = Path(str(self.get_parameter("face_index_path").value or ""))
        metadata_path_param = str(self.get_parameter("face_index_metadata_path").value or "")
        metadata_path = Path(metadata_path_param) if metadata_path_param else None
        face_index = None
        if face_index_path and face_index_path.exists():
            if face_index_path.is_dir():
                self.get_logger().error(
                    "face_index_path parameter must point to a JSON or YAML file, "
                    f"got directory '{face_index_path}'"
                )
            else:
                try:
                    face_index = load_face_index(
                        face_index_path, metadata_path=metadata_path
                    )
                    if face_index is None:
                        self.get_logger().warn(
                            "Face index configuration empty; falling back to heuristics"
                        )
                except Exception as exc:  # pragma: no cover - logging best effort
                    self.get_logger().error(
                        f"Failed to load face index from '{face_index_path}': {exc}"
                    )

        identity_config = FaceIdentityConfig(
            user_similarity_threshold=float(
                self.get_parameter("user_similarity_threshold").value
            ),
            guest_similarity_threshold=float(
                self.get_parameter("guest_similarity_threshold").value
            ),
            unknown_label=str(self.get_parameter("unknown_label").value),
            fallback_to_heuristic_for_unknown=bool(
                self.get_parameter("fallback_to_heuristic_for_unknown").value
            ),
        )
        self._snapshot_timeout = float(self.get_parameter("face_snapshot_timeout").value)

        self._resolver = FaceIdentityResolver(
            classifier=self.classifier,
            index=face_index,
            cache=self._snapshot_cache,
            config=identity_config,
            snapshot_fetcher=self._request_snapshot_via_service,
        )

        snapshot_topic = str(self.get_parameter("face_snapshot_topic").value)
        self._face_subscription = None
        if FaceSnapshotMsg is not None and snapshot_topic:
            self._face_subscription = self.create_subscription(
                FaceSnapshotMsg,
                snapshot_topic,
                self._on_face_snapshot,
                10,
            )

        self._snapshot_client = None
        if GetFaceSnapshot is not None:
            snapshot_service = str(self.get_parameter("face_snapshot_service").value)
            if snapshot_service:
                self._snapshot_client = self.create_client(
                    GetFaceSnapshot,
                    snapshot_service,
                )

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
        decision = self._resolver.resolve(observation, observation.track_id)
        result = decision.result
        response.label = result.label
        response.is_user = result.is_user
        response.confidence = float(result.confidence)
        response.reason = result.reason
        if hasattr(response, "embedding_id") and decision.embedding_id is not None:
            response.embedding_id = decision.embedding_id
        if hasattr(response, "embedding_similarity") and decision.similarity is not None:
            response.embedding_similarity = float(decision.similarity)
        if hasattr(response, "snapshot_age") and decision.snapshot_age is not None:
            response.snapshot_age = float(decision.snapshot_age)
        if hasattr(response, "used_face_embedding"):
            response.used_face_embedding = not decision.used_fallback
        return response

    def _on_face_snapshot(self, msg) -> None:
        try:
            embedding_data = None
            if hasattr(msg, "embedding"):
                embedding = getattr(msg, "embedding")
                if isinstance(embedding, np.ndarray):
                    embedding_data = embedding.astype(np.float32)
                elif hasattr(embedding, "data"):
                    embedding_data = np.asarray(embedding.data, dtype=np.float32)
                else:
                    embedding_data = np.asarray(list(embedding), dtype=np.float32)
            if embedding_data is None or embedding_data.size == 0:
                return
            track_id = int(getattr(msg, "track_id", -1))
            embedding_id = getattr(msg, "embedding_id", None)
            quality = getattr(msg, "quality", None)
            snapshot = FaceSnapshot(
                track_id=track_id,
                embedding=embedding_data,
                embedding_id=str(embedding_id) if embedding_id not in {None, ""} else None,
                quality=float(quality) if isinstance(quality, (float, int)) else None,
            )
            self._resolver.update_snapshot(snapshot)
        except Exception as exc:  # pragma: no cover - logging best effort
            self.get_logger().warn(f"Failed to process face snapshot: {exc}")

    def _request_snapshot_via_service(self, track_id: int) -> Optional[FaceSnapshot]:
        if self._snapshot_client is None:
            return None
        if not self._snapshot_client.service_is_ready():
            return None
        try:
            request = self._snapshot_client.srv_type.Request()  # type: ignore[attr-defined]
        except Exception:  # pragma: no cover - best effort for compatibility
            request = None
        if request is None:
            return None
        if hasattr(request, "track_id"):
            request.track_id = int(track_id)
        future = self._snapshot_client.call_async(request)
        if self._snapshot_timeout > 0.0:
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=self._snapshot_timeout,
            )
        if not future.done():
            return None
        try:
            response = future.result()
        except Exception:  # pragma: no cover - best effort logging
            return None
        snapshot_msg = None
        if hasattr(response, "snapshot"):
            snapshot_msg = response.snapshot
        elif hasattr(response, "snapshots") and response.snapshots:
            snapshot_msg = response.snapshots[0]
        if snapshot_msg is None:
            return None
        try:
            embedding = getattr(snapshot_msg, "embedding")
            if isinstance(embedding, np.ndarray):
                vector = embedding.astype(np.float32)
            elif hasattr(embedding, "data"):
                vector = np.asarray(embedding.data, dtype=np.float32)
            else:
                vector = np.asarray(list(embedding), dtype=np.float32)
            if vector.size == 0:
                return None
            embedding_id = getattr(snapshot_msg, "embedding_id", None)
            quality = getattr(snapshot_msg, "quality", None)
            return FaceSnapshot(
                track_id=int(getattr(snapshot_msg, "track_id", track_id)),
                embedding=vector,
                embedding_id=str(embedding_id) if embedding_id not in {None, ""} else None,
                quality=float(quality) if isinstance(quality, (float, int)) else None,
            )
        except Exception:  # pragma: no cover - best effort logging
            return None


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
