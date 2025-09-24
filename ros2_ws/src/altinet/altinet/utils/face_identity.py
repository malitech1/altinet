"""Face recognition utilities used by the identity service."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .config import load_file


ArrayLike = Sequence[float] | np.ndarray


@dataclass
class FaceSnapshot:
    """Container for a normalised face embedding tied to a track."""

    track_id: int
    embedding: np.ndarray
    embedding_id: Optional[str] = None
    quality: Optional[float] = None

    def __post_init__(self) -> None:
        vector = np.asarray(self.embedding, dtype=np.float32)
        if vector.ndim != 1:
            raise ValueError("FaceSnapshot embedding must be a 1D vector")
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm <= 0.0:
            self.embedding = np.zeros_like(vector)
        else:
            self.embedding = vector / norm


class FaceSnapshotCache:
    """LRU-style cache storing the most recent snapshot per track."""

    def __init__(
        self,
        max_age_sec: float,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.max_age_sec = max(0.0, float(max_age_sec))
        self._clock = clock or time.monotonic
        self._entries: Dict[int, Tuple[FaceSnapshot, float]] = {}

    def update(self, snapshot: FaceSnapshot) -> None:
        """Store ``snapshot`` and mark it as freshly seen."""

        self._entries[int(snapshot.track_id)] = (snapshot, self._clock())

    def get(self, track_id: int) -> Tuple[Optional[FaceSnapshot], Optional[float]]:
        """Return the cached snapshot for ``track_id`` and its age."""

        entry = self._entries.get(int(track_id))
        if entry is None:
            return None, None
        snapshot, timestamp = entry
        age = self._clock() - timestamp
        if self.max_age_sec > 0.0 and age > self.max_age_sec:
            self._entries.pop(int(track_id), None)
            return None, None
        return snapshot, max(0.0, age)

    def prune(self) -> None:
        """Remove expired entries eagerly."""

        expired: List[int] = []
        now = self._clock()
        if self.max_age_sec <= 0.0:
            return
        for track_id, (_, stored) in self._entries.items():
            if now - stored > self.max_age_sec:
                expired.append(track_id)
        for track_id in expired:
            self._entries.pop(track_id, None)


@dataclass
class FaceMatch:
    """Represents a similarity match returned by :class:`FaceEmbeddingIndex`."""

    label: str
    similarity: float
    embedding_id: Optional[str] = None
    index: int | None = None


class FaceEmbeddingIndex:
    """Simple cosine-similarity index backed by NumPy arrays."""

    def __init__(
        self,
        embeddings: ArrayLike,
        labels: Sequence[str],
        *,
        embedding_ids: Sequence[str] | None = None,
        label_is_user: Mapping[str, bool] | None = None,
    ) -> None:
        vectors = np.asarray(embeddings, dtype=np.float32)
        if vectors.ndim != 2:
            raise ValueError("embeddings must be a 2D array")
        if len(labels) != vectors.shape[0]:
            raise ValueError("labels length must match number of embeddings")
        norms = np.linalg.norm(vectors, axis=1)
        norms[norms == 0] = 1.0
        self._vectors = vectors / norms[:, np.newaxis]
        self._labels = list(labels)
        if embedding_ids is not None and len(embedding_ids) != len(labels):
            raise ValueError("embedding_ids length must match labels length")
        self._embedding_ids = list(embedding_ids) if embedding_ids is not None else list(labels)
        self._label_is_user = {label: bool(value) for label, value in (label_is_user or {}).items()}

    @property
    def dimension(self) -> int:
        """Return the dimensionality of the index."""

        return int(self._vectors.shape[1]) if self._vectors.size else 0

    def __len__(self) -> int:  # pragma: no cover - trivial
        return int(self._vectors.shape[0])

    def is_user(self, label: str) -> bool:
        """Return whether ``label`` represents a known user."""

        return bool(self._label_is_user.get(label, False))

    def search(self, embedding: ArrayLike, *, limit: int = 1) -> List[FaceMatch]:
        """Return the most similar labels for ``embedding``."""

        if self._vectors.size == 0:
            return []
        vector = np.asarray(embedding, dtype=np.float32)
        if vector.ndim != 1:
            raise ValueError("embedding must be 1D")
        if vector.shape[0] != self._vectors.shape[1]:
            raise ValueError("embedding dimension mismatch")
        norm = float(np.linalg.norm(vector))
        if not math.isfinite(norm) or norm == 0.0:
            return []
        normalised = vector / norm
        similarities = self._vectors @ normalised
        order = np.argsort(similarities)[::-1]
        results: List[FaceMatch] = []
        for rank in order[: max(1, limit)]:
            similarity = float(similarities[rank])
            results.append(
                FaceMatch(
                    label=self._labels[int(rank)],
                    similarity=similarity,
                    embedding_id=self._embedding_ids[int(rank)],
                    index=int(rank),
                )
            )
        return results


@dataclass
class FaceIdentityConfig:
    """Configuration values controlling face identity resolution."""

    user_similarity_threshold: float = 0.72
    guest_similarity_threshold: float = 0.62
    unknown_label: str = "unknown"
    fallback_to_heuristic_for_unknown: bool = False


@dataclass
class FaceIdentityDecision:
    """Outcome returned by :class:`FaceIdentityResolver`."""

    result: "IdentityResult"
    embedding_id: Optional[str] = None
    similarity: Optional[float] = None
    snapshot_age: Optional[float] = None
    used_fallback: bool = False


class FaceIdentityResolver:
    """Combine face embeddings with heuristics to classify people."""

    def __init__(
        self,
        classifier: "IdentityClassifier",
        index: Optional[FaceEmbeddingIndex],
        cache: FaceSnapshotCache,
        config: FaceIdentityConfig,
        *,
        snapshot_fetcher: Callable[[int], Optional[FaceSnapshot]] | None = None,
    ) -> None:
        self.classifier = classifier
        self.index = index
        self.cache = cache
        self.config = config
        self._snapshot_fetcher = snapshot_fetcher

    def update_snapshot(self, snapshot: FaceSnapshot) -> None:
        """Update the cached snapshot for the corresponding track."""

        self.cache.update(snapshot)

    def resolve(
        self,
        observation: "IdentityObservation",
        track_id: int,
    ) -> FaceIdentityDecision:
        """Return an identity decision for ``track_id``."""

        snapshot: Optional[FaceSnapshot]
        age: Optional[float]
        snapshot, age = self.cache.get(track_id)
        if snapshot is None and track_id >= 0 and self._snapshot_fetcher is not None:
            fetched = self._snapshot_fetcher(track_id)
            if fetched is not None:
                self.cache.update(fetched)
                snapshot, age = self.cache.get(track_id)

        if snapshot is None or self.index is None or len(self.index) == 0:
            fallback = self.classifier.classify(observation)
            reason = "No face embedding available"
            if track_id >= 0:
                reason += f" for track {track_id}"
            if fallback.reason:
                reason = f"{reason}; {fallback.reason}"
            result = IdentityResult(
                label=fallback.label,
                is_user=fallback.is_user,
                confidence=fallback.confidence,
                reason=reason,
            )
            return FaceIdentityDecision(
                result=result,
                used_fallback=True,
            )

        matches = self.index.search(snapshot.embedding, limit=1)
        if not matches:
            return self._unknown_result(observation, track_id, age, similarity=None)

        match = matches[0]
        is_user = self.index.is_user(match.label)
        threshold = (
            self.config.user_similarity_threshold if is_user else self.config.guest_similarity_threshold
        )
        if match.similarity >= threshold:
            reason = (
                f"cosine similarity {match.similarity:.3f} >= "
                f"threshold {threshold:.3f} for label '{match.label}'"
            )
            confidence = float(np.clip(match.similarity, 0.0, 1.0))
            result = IdentityResult(
                label=match.label,
                is_user=is_user,
                confidence=confidence,
                reason=reason,
            )
            return FaceIdentityDecision(
                result=result,
                embedding_id=match.embedding_id,
                similarity=match.similarity,
                snapshot_age=age,
                used_fallback=False,
            )

        if self.config.fallback_to_heuristic_for_unknown:
            fallback = self.classifier.classify(observation)
            reason = (
                f"Face similarity {match.similarity:.3f} below threshold "
                f"{threshold:.3f}; {fallback.reason}"
            )
            result = IdentityResult(
                label=fallback.label,
                is_user=fallback.is_user,
                confidence=fallback.confidence,
                reason=reason,
            )
            return FaceIdentityDecision(
                result=result,
                embedding_id=match.embedding_id,
                similarity=match.similarity,
                snapshot_age=age,
                used_fallback=True,
            )

        return self._unknown_result(observation, track_id, age, match.similarity, match.embedding_id)

    def _unknown_result(
        self,
        observation: "IdentityObservation",
        track_id: int,
        age: Optional[float],
        similarity: Optional[float],
        embedding_id: Optional[str] = None,
    ) -> FaceIdentityDecision:
        similarity_value = float(similarity) if similarity is not None else 0.0
        similarity_value = float(np.clip(similarity_value, 0.0, 1.0))
        reason = "Face embedding below similarity threshold"
        if track_id >= 0:
            reason += f" for track {track_id}"
        if similarity is not None:
            reason += f" (score {similarity:.3f})"
        result = IdentityResult(
            label=self.config.unknown_label,
            is_user=False,
            confidence=similarity_value,
            reason=reason,
        )
        return FaceIdentityDecision(
            result=result,
            embedding_id=embedding_id,
            similarity=similarity,
            snapshot_age=age,
            used_fallback=False,
        )


def load_face_index(
    path: Optional[Path],
    *,
    metadata_path: Optional[Path] = None,
) -> Optional[FaceEmbeddingIndex]:
    """Load a :class:`FaceEmbeddingIndex` from JSON/YAML configuration files."""

    if path is None:
        return None
    if not str(path):
        return None
    index_path = Path(path)
    if not index_path.exists():
        raise FileNotFoundError(f"Face index path '{index_path}' does not exist")
    data = load_file(index_path)
    records: Iterable[Mapping[str, object]]
    if "records" in data:
        records = list(data["records"] or [])
    else:
        labels = data.get("labels") or []
        embeddings = data.get("embeddings") or []
        if len(labels) != len(embeddings):
            raise ValueError("labels and embeddings length mismatch in face index config")
        records = [
            {"label": label, "embedding": emb}
            for label, emb in zip(labels, embeddings)
        ]
        if not records:
            return None
    embeddings_list: List[np.ndarray] = []
    labels: List[str] = []
    embedding_ids: List[str] = []
    is_user_map: Dict[str, bool] = {}
    for record in records:
        label = str(record.get("label"))
        embedding = np.asarray(record.get("embedding"), dtype=np.float32)
        if embedding.ndim != 1:
            raise ValueError("face embedding entries must be 1D vectors")
        embeddings_list.append(embedding)
        labels.append(label)
        embedding_ids.append(str(record.get("embedding_id", label)))
        if "is_user" in record:
            is_user_map[label] = bool(record.get("is_user"))

    if not embeddings_list:
        return None

    if metadata_path is not None and str(metadata_path):
        meta_file = Path(metadata_path)
        if meta_file.exists():
            metadata = load_file(meta_file)
            if isinstance(metadata, Mapping):
                for label, value in metadata.get("label_metadata", {}).items():
                    if isinstance(value, Mapping) and "is_user" in value:
                        is_user_map[str(label)] = bool(value["is_user"])

    return FaceEmbeddingIndex(
        embeddings=np.vstack(embeddings_list),
        labels=labels,
        embedding_ids=embedding_ids,
        label_is_user=is_user_map,
    )


from .identity import IdentityClassifier, IdentityObservation, IdentityResult


__all__ = [
    "FaceEmbeddingIndex",
    "FaceIdentityConfig",
    "FaceIdentityDecision",
    "FaceIdentityResolver",
    "FaceSnapshot",
    "FaceSnapshotCache",
    "FaceMatch",
    "load_face_index",
]

