"""Lightweight face embedding gallery manager used by tooling and ROS nodes."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, MutableMapping, Optional, Sequence, Tuple
import uuid

import numpy as np

try:  # pragma: no cover - optional dependency used when available
    import face_recognition  # type: ignore
except ImportError:  # pragma: no cover - gracefully degrade in environments without dlib
    face_recognition = None  # type: ignore


@dataclass
class SnapshotEntry:
    """Metadata about a single enrolled or rejected sample."""

    embedding_id: Optional[str]
    quality: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        payload = {"quality": float(self.quality), **self.metadata}
        if self.embedding_id:
            payload["embedding_id"] = self.embedding_id
        return payload


@dataclass
class EnrollmentResult:
    """Result returned after attempting to enrol face samples."""

    identity_id: str
    accepted: List[SnapshotEntry] = field(default_factory=list)
    rejected: List[SnapshotEntry] = field(default_factory=list)
    quality_threshold: float = 0.0

    @property
    def accepted_count(self) -> int:
        return len(self.accepted)

    @property
    def rejected_count(self) -> int:
        return len(self.rejected)


def _as_numpy(image: np.ndarray | Sequence[Any]) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype == np.uint8:
        return array.astype(np.float32) / 255.0
    return array.astype(np.float32)


def _quality_score(image: np.ndarray) -> float:
    if image.size == 0:
        return 0.0
    if image.ndim == 3:
        image = image.mean(axis=2)
    return float(np.clip(np.std(image), 0.0, 1.0))


def _compute_embedding(image: np.ndarray, embedding_dim: int) -> np.ndarray:
    flat = image.mean(axis=2) if image.ndim == 3 else image
    flat = flat.flatten()
    if flat.size == 0:
        return np.zeros(embedding_dim, dtype=np.float32)
    if flat.size < embedding_dim:
        padded = np.zeros(embedding_dim, dtype=np.float32)
        padded[: flat.size] = flat
        vector = padded
    else:
        vector = flat[:embedding_dim].astype(np.float32)
    norm = np.linalg.norm(vector)
    if norm > 0:
        vector = vector / norm
    return vector.astype(np.float32)


def _prepare_vector(values: Sequence[float], embedding_dim: int) -> np.ndarray:
    array = np.asarray(list(values), dtype=np.float32)
    if array.size < embedding_dim:
        padded = np.zeros(embedding_dim, dtype=np.float32)
        padded[: array.size] = array
        array = padded
    elif array.size > embedding_dim:
        array = array[:embedding_dim]
    norm = np.linalg.norm(array)
    if norm > 0:
        array = array / norm
    return array.astype(np.float32)


class FaceRecognitionService:
    """Maintain a gallery of face embeddings with on-disk persistence."""

    def __init__(
        self,
        gallery_dir: str | os.PathLike[str] | None = None,
        *,
        embedding_dim: int = 128,
    ) -> None:
        self.gallery_dir = Path(gallery_dir or Path("assets/face_gallery"))
        self.embedding_dim = int(embedding_dim)
        self.index_path = self.gallery_dir / "gallery.faiss"
        self.npy_path = self.gallery_dir / "gallery.npy"
        self.metadata_path = self.gallery_dir / "metadata.json"
        self.snapshots_dir = self.gallery_dir / "snapshots"
        self._embeddings: np.ndarray = np.zeros((0, self.embedding_dim), dtype=np.float32)
        self._metadata: List[Dict[str, Any]] = []
        self._listeners: List[Callable[[EnrollmentResult], None]] = []
        self._load()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def register_listener(self, callback: Callable[[EnrollmentResult], None]) -> None:
        """Register ``callback`` to be invoked after successful enrolments."""

        self._listeners.append(callback)

    def enrol_samples(
        self,
        identity_id: str,
        samples: Sequence[Tuple[np.ndarray, MutableMapping[str, Any]]],
        *,
        quality_threshold: float = 0.0,
        track_id: Optional[int] = None,
        camera_id: Optional[str] = None,
        extra_metadata: Optional[Dict[str, Any]] = None,
    ) -> EnrollmentResult:
        """Enrol ``samples`` for ``identity_id`` applying ``quality_threshold``.

        ``samples`` should be an iterable of ``(image, metadata)`` tuples where
        ``image`` is convertible to a :class:`numpy.ndarray`.
        """

        accepted: List[SnapshotEntry] = []
        rejected: List[SnapshotEntry] = []
        new_embeddings: List[np.ndarray] = []
        new_metadata: List[Dict[str, Any]] = []
        extra_metadata = extra_metadata or {}

        for image, metadata in samples:
            array = _as_numpy(image)
            quality = _quality_score(array)
            combined_meta: Dict[str, Any] = {**extra_metadata, **metadata}
            combined_meta["camera_id"] = camera_id
            combined_meta["track_id"] = track_id
            combined_meta.setdefault("captured_at", datetime.utcnow().isoformat())
            combined_meta["quality"] = float(quality)
            if quality < quality_threshold:
                combined_meta["reason"] = "quality_below_threshold"
                rejected.append(
                    SnapshotEntry(embedding_id=None, quality=quality, metadata=combined_meta)
                )
                continue

            embedding_id = str(uuid.uuid4())
            embedding = _compute_embedding(array, self.embedding_dim)
            new_embeddings.append(embedding)
            new_metadata.append(
                {
                    "embedding_id": embedding_id,
                    "identity_id": identity_id,
                    "camera_id": camera_id,
                    "track_id": track_id,
                    "captured_at": combined_meta["captured_at"],
                    "quality": float(quality),
                    "metadata": combined_meta,
                }
            )
            combined_meta["embedding_id"] = embedding_id
            accepted.append(
                SnapshotEntry(embedding_id=embedding_id, quality=quality, metadata=combined_meta)
            )

        result = EnrollmentResult(
            identity_id=identity_id,
            accepted=accepted,
            rejected=rejected,
            quality_threshold=quality_threshold,
        )

        if new_embeddings:
            self._append_embeddings(new_embeddings, new_metadata)
            self._write_snapshot(result)
            self._notify_listeners(result)
        else:
            # Still capture a snapshot for auditability even when nothing was added
            self._write_snapshot(result)

        return result

    def enrol_directory(
        self,
        identity_id: str,
        directory: str | os.PathLike[str],
        *,
        pattern: str = "*.jpg",
        quality_threshold: float = 0.0,
        track_id: Optional[int] = None,
        camera_id: Optional[str] = None,
        extra_metadata: Optional[Dict[str, Any]] = None,
    ) -> EnrollmentResult:
        """Load images from ``directory`` and enrol them."""

        directory_path = Path(directory)
        samples: List[Tuple[np.ndarray, MutableMapping[str, Any]]] = []
        if not directory_path.exists():
            return EnrollmentResult(identity_id=identity_id)

        for path in sorted(directory_path.glob(pattern)):
            image = self._load_image(path)
            if image is None:
                continue
            samples.append((image, {"source_path": str(path)}))

        return self.enrol_samples(
            identity_id,
            samples,
            quality_threshold=quality_threshold,
            track_id=track_id,
            camera_id=camera_id,
            extra_metadata=extra_metadata,
        )

    def enrol_embeddings(
        self,
        identity_id: str,
        vectors: Sequence[Tuple[Sequence[float], MutableMapping[str, Any]]],
        *,
        track_id: Optional[int] = None,
        camera_id: Optional[str] = None,
        quality_default: float = 1.0,
    ) -> EnrollmentResult:
        """Persist precomputed face embeddings into the gallery."""

        accepted: List[SnapshotEntry] = []
        new_embeddings: List[np.ndarray] = []
        new_metadata: List[Dict[str, Any]] = []

        for vector, metadata in vectors:
            combined_meta: Dict[str, Any] = dict(metadata)
            combined_meta["camera_id"] = camera_id
            combined_meta["track_id"] = track_id
            combined_meta.setdefault("captured_at", datetime.utcnow().isoformat())
            quality = float(combined_meta.get("quality", quality_default))
            combined_meta["quality"] = quality
            embedding_id = str(uuid.uuid4())
            embedding = _prepare_vector(vector, self.embedding_dim)
            new_embeddings.append(embedding)
            new_metadata.append(
                {
                    "embedding_id": embedding_id,
                    "identity_id": identity_id,
                    "camera_id": camera_id,
                    "track_id": track_id,
                    "captured_at": combined_meta["captured_at"],
                    "quality": quality,
                    "metadata": combined_meta,
                }
            )
            combined_meta["embedding_id"] = embedding_id
            accepted.append(
                SnapshotEntry(embedding_id=embedding_id, quality=quality, metadata=combined_meta)
            )

        result = EnrollmentResult(identity_id=identity_id, accepted=accepted, rejected=[])
        if new_embeddings:
            self._append_embeddings(new_embeddings, new_metadata)
            self._write_snapshot(result)
            self._notify_listeners(result)
        return result

    def as_index(self) -> np.ndarray:
        """Return a copy of the current embedding matrix."""

        return np.array(self._embeddings, copy=True)

    def metadata(self) -> List[Dict[str, Any]]:
        """Return a deep copy of the metadata list."""

        return json.loads(json.dumps(self._metadata))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _load(self) -> None:
        if self.npy_path.exists():
            self._embeddings = np.load(self.npy_path, allow_pickle=False)
        if self.metadata_path.exists():
            with self.metadata_path.open("r", encoding="utf-8") as fh:
                self._metadata = json.load(fh)

    def _append_embeddings(
        self, embeddings: Sequence[np.ndarray], metadata: Sequence[Dict[str, Any]]
    ) -> None:
        matrix = np.vstack(embeddings).astype(np.float32)
        if self._embeddings.size == 0:
            self._embeddings = matrix
        else:
            self._embeddings = np.vstack([self._embeddings, matrix])
        self._metadata.extend(metadata)
        self._persist()

    def _persist(self) -> None:
        self.gallery_dir.mkdir(parents=True, exist_ok=True)
        self.snapshots_dir.mkdir(parents=True, exist_ok=True)
        self._atomic_write(
            self.npy_path, lambda fh: np.save(fh, self._embeddings), binary=True
        )
        # Mirror the contents to a .faiss file for compatibility with downstream tooling
        self._atomic_write(
            self.index_path, lambda fh: np.save(fh, self._embeddings), binary=True
        )
        self._atomic_write(
            self.metadata_path,
            lambda fh: json.dump(self._metadata, fh, indent=2, sort_keys=True),
        )

    def _write_snapshot(self, result: EnrollmentResult) -> None:
        snapshot_name = (
            datetime.utcnow().strftime("%Y%m%dT%H%M%S%fZ") + f"_{result.identity_id}.json"
        )
        snapshot_path = self.snapshots_dir / snapshot_name
        payload = {
            "identity_id": result.identity_id,
            "quality_threshold": float(result.quality_threshold),
            "accepted": [entry.as_dict() for entry in result.accepted],
            "rejected": [entry.as_dict() for entry in result.rejected],
            "created_at": datetime.utcnow().isoformat(),
        }
        self._atomic_write(snapshot_path, lambda fh: json.dump(payload, fh, indent=2))

    def _notify_listeners(self, result: EnrollmentResult) -> None:
        for callback in list(self._listeners):
            try:
                callback(result)
            except Exception:  # pragma: no cover - defensive, listeners should be safe
                continue

    def _load_image(self, path: Path) -> Optional[np.ndarray]:
        if face_recognition is not None:
            try:
                return face_recognition.load_image_file(path)
            except Exception:  # pragma: no cover - dependency specific failure
                return None
        try:
            return np.load(path)
        except Exception:
            return None

    @staticmethod
    def _atomic_write(path: Path, writer: Callable[[Any], None], *, binary: bool = False) -> None:
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        kwargs: Dict[str, Any] = {}
        mode = "wb" if binary else "w"
        if not binary:
            kwargs["encoding"] = "utf-8"
        with tmp_path.open(mode, **kwargs) as fh:
            writer(fh)
        os.replace(tmp_path, path)


__all__ = [
    "EnrollmentResult",
    "FaceRecognitionService",
    "SnapshotEntry",
]
