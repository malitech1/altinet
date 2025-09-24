"""Face recognition utilities for Altinet."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np

try:  # pragma: no cover - optional dependency for runtime only
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - OpenCV not required for tests
    cv2 = None

try:  # pragma: no cover - heavy dependency
    import onnxruntime as ort  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    ort = None

try:  # pragma: no cover - heavy dependency
    import insightface  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    insightface = None

try:  # pragma: no cover - heavy dependency
    import faiss  # type: ignore
except ImportError:  # pragma: no cover - handled at runtime
    faiss = None

from .types import BoundingBox

# Canonical landmark template used by ArcFace-style embedders.
ARCFACE_TEMPLATE = np.array(
    [
        [38.2946, 51.6963],
        [73.5318, 51.5014],
        [56.0252, 71.7366],
        [41.5493, 92.3655],
        [70.7299, 92.2041],
    ],
    dtype=np.float32,
)


@dataclass
class FaceQualityConfig:
    """Optional thresholds used to filter detections."""

    enabled: bool = False
    max_roll: float = 35.0
    max_yaw: float = 35.0
    max_pitch: float = 35.0
    min_sharpness: float = 10.0
    max_occlusion: float = 0.6


@dataclass
class RetinaFaceConfig:
    """Configuration used to run RetinaFace detectors."""

    model_path: Path
    det_thresh: float = 0.5
    nms_thresh: float = 0.4
    input_size: Tuple[int, int] = (640, 640)
    providers: Optional[Sequence[str]] = None
    max_faces: int = 32
    quality: Optional[FaceQualityConfig] = None


@dataclass
class FaceEmbedderConfig:
    """Configuration for ArcFace/AdaFace style embedders."""

    model_path: Path
    input_size: Tuple[int, int] = (112, 112)
    mean: Sequence[float] = (127.5, 127.5, 127.5)
    std: Sequence[float] = (128.0, 128.0, 128.0)
    providers: Optional[Sequence[str]] = None
    embedding_dim: int = 512
    model_type: str = "arcface"


@dataclass
class FaceIdentifierConfig:
    """Configuration for FAISS backed face identification."""

    embedding_dim: int = 512
    similarity_threshold: float = 0.5
    top_k: int = 1
    normalize: bool = True


@dataclass
class FaceQuality:
    """Represents heuristic quality scores for a face detection."""

    roll: float
    yaw: float
    pitch: float
    sharpness: float
    occlusion: float
    passed: bool


@dataclass
class FaceDetection:
    """Normalised RetinaFace detection output."""

    bbox: BoundingBox
    confidence: float
    landmarks: np.ndarray
    quality: FaceQuality


@dataclass
class AlignedFace:
    """Aligned face crop ready for embedding."""

    image: np.ndarray
    transform: np.ndarray
    landmarks: np.ndarray


class RetinaFaceDetector:
    """Run RetinaFace detection and compute quality heuristics."""

    def __init__(
        self,
        config: RetinaFaceConfig,
        analysis: Optional[Any] = None,
    ) -> None:
        self.config = config
        self.quality_config = config.quality or FaceQualityConfig()
        if analysis is not None:
            self._analysis = analysis
        else:
            if insightface is None:
                raise RuntimeError(
                    "insightface is required to use RetinaFaceDetector. Install insightface to enable detection."
                )
            providers = list(config.providers or [])
            kwargs: dict[str, Any] = {}
            if providers:
                kwargs["providers"] = providers
            self._analysis = insightface.app.FaceAnalysis(  # type: ignore[attr-defined]
                allowed_modules=["detection"],
                **kwargs,
            )
            self._analysis.prepare(
                ctx_id=-1,
                det_thresh=float(config.det_thresh),
                det_size=config.input_size,
            )
            if config.model_path:
                try:
                    self._analysis.models["detection"].prepare(ctx_id=-1, det_size=config.input_size)
                except Exception:  # pragma: no cover - depends on runtime insightface internals
                    pass

    def detect(self, image: np.ndarray) -> List[FaceDetection]:
        """Return RetinaFace detections for ``image``."""

        faces = self._analysis.get(image)  # type: ignore[attr-defined]
        detections: List[FaceDetection] = []
        for face in faces[: self.config.max_faces]:
            bbox = _bbox_from_array(np.asarray(face.bbox, dtype=np.float32))
            landmarks = np.asarray(face.kps, dtype=np.float32)
            confidence = float(getattr(face, "det_score", 0.0))
            quality = self._evaluate_quality(image, bbox, landmarks)
            detections.append(
                FaceDetection(
                    bbox=bbox,
                    confidence=confidence,
                    landmarks=landmarks,
                    quality=quality,
                )
            )
        return detections

    def _evaluate_quality(
        self, image: np.ndarray, bbox: BoundingBox, landmarks: np.ndarray
    ) -> FaceQuality:
        roll, yaw, pitch = _estimate_pose(landmarks)
        sharpness = _estimate_sharpness(image, bbox)
        occlusion = _estimate_occlusion_ratio(bbox, landmarks)
        passed = True
        if self.quality_config.enabled:
            passed = (
                abs(roll) <= self.quality_config.max_roll
                and abs(yaw) <= self.quality_config.max_yaw
                and abs(pitch) <= self.quality_config.max_pitch
                and sharpness >= self.quality_config.min_sharpness
                and occlusion <= self.quality_config.max_occlusion
            )
        return FaceQuality(
            roll=float(roll),
            yaw=float(yaw),
            pitch=float(pitch),
            sharpness=float(sharpness),
            occlusion=float(occlusion),
            passed=passed,
        )


class FaceEmbedder:
    """Generate feature embeddings using ArcFace/AdaFace models."""

    def __init__(
        self,
        config: FaceEmbedderConfig,
        session: Optional[Any] = None,
    ) -> None:
        self.config = config
        if session is not None:
            self.session = session
        else:
            if ort is None:
                raise RuntimeError(
                    "onnxruntime is required to run the face embedder. Install onnxruntime or onnxruntime-gpu."
                )
            providers = list(config.providers or ort.get_available_providers())
            self.session = ort.InferenceSession(str(config.model_path), providers=providers)
        inputs = self.session.get_inputs()
        if not inputs:
            raise RuntimeError("Embedder session has no inputs defined")
        self.input_name = inputs[0].name

    def embed(self, face: np.ndarray) -> np.ndarray:
        """Return a normalised embedding for ``face``."""

        blob = self._preprocess(face)
        outputs = self.session.run(None, {self.input_name: blob})
        if not outputs:
            raise RuntimeError("Embedder session produced no outputs")
        embedding = np.asarray(outputs[0]).squeeze()
        if embedding.ndim != 1:
            embedding = embedding.reshape(-1)
        return normalize_embedding(embedding)

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        input_h, input_w = self.config.input_size
        resized = _resize_image(image, (input_h, input_w))
        array = resized.astype(np.float32)
        mean = np.asarray(self.config.mean, dtype=np.float32).reshape(1, 1, -1)
        std = np.asarray(self.config.std, dtype=np.float32).reshape(1, 1, -1)
        if array.ndim == 2:
            array = array[:, :, np.newaxis]
        if array.shape[2] == 1 and mean.shape[2] == 3:
            array = np.repeat(array, 3, axis=2)
        array = (array - mean) / std
        array = array.transpose(2, 0, 1)
        return array[np.newaxis, ...]


@dataclass
class FaceMatch:
    """Result returned by :class:`FaceIdentifier`."""

    label: str
    score: float
    index: int


class FaceIdentifier:
    """Perform similarity search over registered face embeddings."""

    def __init__(
        self,
        config: FaceIdentifierConfig,
        index: Optional[Any] = None,
    ) -> None:
        self.config = config
        self.labels: List[str] = []
        if index is not None:
            self.index = index
        else:
            if faiss is None:
                raise RuntimeError(
                    "faiss-cpu is required to use FaceIdentifier. Install faiss-cpu or faiss-gpu."
                )
            self.index = faiss.IndexFlatIP(config.embedding_dim)  # type: ignore[attr-defined]
        self.embedding_dim = int(config.embedding_dim)

    def add(self, label: str, embedding: np.ndarray) -> None:
        """Register ``embedding`` for ``label``."""

        vector = np.asarray(embedding, dtype=np.float32)
        if vector.ndim != 1 or vector.shape[0] != self.embedding_dim:
            raise ValueError(
                f"Embedding must be 1-D with length {self.embedding_dim}, got shape {vector.shape}"
            )
        if self.config.normalize:
            vector = normalize_embedding(vector)
        self.index.add(vector.reshape(1, -1))
        self.labels.append(label)

    def identify(self, embedding: np.ndarray) -> Optional[FaceMatch]:
        """Return the best match above threshold for ``embedding``."""

        if not self.labels:
            return None
        vector = np.asarray(embedding, dtype=np.float32)
        if vector.ndim != 1 or vector.shape[0] != self.embedding_dim:
            raise ValueError(
                f"Embedding must be 1-D with length {self.embedding_dim}, got shape {vector.shape}"
            )
        if self.config.normalize:
            vector = normalize_embedding(vector)
        top_k = max(1, int(self.config.top_k))
        scores, indices = self.index.search(vector.reshape(1, -1), top_k)
        best_score = float(scores[0][0]) if scores.size else -1.0
        best_idx = int(indices[0][0]) if indices.size else -1
        if best_idx < 0 or best_idx >= len(self.labels):
            return None
        if best_score < self.config.similarity_threshold:
            return None
        return FaceMatch(label=self.labels[best_idx], score=best_score, index=best_idx)


def align_face(
    image: np.ndarray,
    landmarks: np.ndarray,
    output_size: Tuple[int, int] = (112, 112),
    template: np.ndarray = ARCFACE_TEMPLATE,
) -> AlignedFace:
    """Align ``image`` using ``landmarks`` and return the crop."""

    if landmarks.shape != (5, 2):
        raise ValueError("Landmarks must be of shape (5, 2)")
    template = np.asarray(template, dtype=np.float32)
    if template.shape != (5, 2):
        raise ValueError("Template must be of shape (5, 2)")
    transform = _estimate_similarity_transform(landmarks, template)
    warp = transform[:2, :]
    aligned = _warp_affine(image, warp, output_size)
    return AlignedFace(image=aligned, transform=transform, landmarks=landmarks.copy())


def normalize_embedding(embedding: np.ndarray) -> np.ndarray:
    """Return the L2 normalised ``embedding`` (no-op for zero vectors)."""

    vector = np.asarray(embedding, dtype=np.float32)
    if vector.ndim != 1:
        vector = vector.reshape(-1)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        return np.zeros_like(vector)
    return vector / norm


def _bbox_from_array(array: np.ndarray) -> BoundingBox:
    x1, y1, x2, y2 = map(float, array)
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    return BoundingBox(x=x1, y=y1, w=width, h=height)


def _estimate_pose(landmarks: np.ndarray) -> Tuple[float, float, float]:
    left_eye, right_eye, nose, left_mouth, right_mouth = landmarks
    eye_vector = right_eye - left_eye
    roll = math.degrees(math.atan2(eye_vector[1], eye_vector[0] + 1e-6))
    mid_eye = (left_eye + right_eye) / 2.0
    eye_distance = np.linalg.norm(eye_vector) + 1e-6
    yaw = math.degrees(math.atan2(nose[0] - mid_eye[0], eye_distance))
    mouth_mid = (left_mouth + right_mouth) / 2.0
    vertical_dist = (mouth_mid[1] - mid_eye[1]) + 1e-6
    pitch = math.degrees(math.atan2(nose[1] - mid_eye[1], vertical_dist))
    return float(roll), float(yaw), float(pitch)


def _estimate_sharpness(image: np.ndarray, bbox: BoundingBox) -> float:
    x1 = max(int(math.floor(bbox.x)), 0)
    y1 = max(int(math.floor(bbox.y)), 0)
    x2 = min(int(math.ceil(bbox.x + bbox.w)), image.shape[1])
    y2 = min(int(math.ceil(bbox.y + bbox.h)), image.shape[0])
    if x2 <= x1 or y2 <= y1:
        return 0.0
    crop = image[y1:y2, x1:x2]
    if crop.ndim == 3:
        crop = crop.mean(axis=2)
    crop = crop.astype(np.float32)
    if crop.shape[0] < 3 or crop.shape[1] < 3:
        return 0.0
    laplacian = (
        crop[0:-2, 1:-1]
        + crop[2:, 1:-1]
        + crop[1:-1, 0:-2]
        + crop[1:-1, 2:]
        - 4 * crop[1:-1, 1:-1]
    )
    return float(np.var(laplacian))


def _estimate_occlusion_ratio(bbox: BoundingBox, landmarks: np.ndarray) -> float:
    centroid = landmarks.mean(axis=0)
    vectors = landmarks - centroid
    angles = np.arctan2(vectors[:, 1], vectors[:, 0])
    order = np.argsort(angles)
    polygon = landmarks[order]
    x = polygon[:, 0]
    y = polygon[:, 1]
    area = 0.5 * float(np.abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    bbox_area = max(bbox.w * bbox.h, 1e-6)
    coverage = min(area / bbox_area, 1.0)
    return float(1.0 - coverage)


def _estimate_similarity_transform(
    source: np.ndarray, target: np.ndarray
) -> np.ndarray:
    if source.shape != (5, 2) or target.shape != (5, 2):
        raise ValueError("source and target must be shaped (5, 2)")
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    src_centered = source - src_mean
    tgt_centered = target - tgt_mean
    cov = (tgt_centered.T @ src_centered) / source.shape[0]
    u, s, vh = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(u) * np.linalg.det(vh))
    s_matrix = np.diag([1.0, d])
    rotation = u @ s_matrix @ vh
    var_src = np.sum(src_centered**2) / source.shape[0]
    scale = float(np.sum(s * np.diag(s_matrix)) / (var_src + 1e-6))
    transform = np.eye(3, dtype=np.float32)
    transform[:2, :2] = scale * rotation
    transform[:2, 2] = tgt_mean - scale * rotation @ src_mean
    return transform


def _warp_affine(image: np.ndarray, matrix: np.ndarray, output_size: Tuple[int, int]) -> np.ndarray:
    height, width = output_size
    if cv2 is not None:
        return cv2.warpAffine(image, matrix, (width, height), flags=cv2.INTER_LINEAR)  # type: ignore[arg-type]
    full = np.vstack([matrix, np.array([0.0, 0.0, 1.0], dtype=np.float32)])
    grid_y, grid_x = np.indices((height, width), dtype=np.float32)
    ones = np.ones_like(grid_x)
    dest = np.stack([grid_x, grid_y, ones], axis=-1)
    src = dest @ full.T
    xs = src[..., 0]
    ys = src[..., 1]
    return _sample_bilinear(image, xs, ys)


def _resize_image(image: np.ndarray, size: Tuple[int, int]) -> np.ndarray:
    height, width = size
    if image.shape[0] == height and image.shape[1] == width:
        return image
    if cv2 is not None:
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
    grid_y = np.linspace(0, image.shape[0] - 1, num=height)
    grid_x = np.linspace(0, image.shape[1] - 1, num=width)
    xs, ys = np.meshgrid(grid_x, grid_y)
    return _sample_bilinear(image, xs, ys)


def _sample_bilinear(image: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    xs = np.clip(xs, 0.0, w - 1.0)
    ys = np.clip(ys, 0.0, h - 1.0)
    x0 = np.floor(xs).astype(np.int32)
    x1 = np.minimum(x0 + 1, w - 1)
    y0 = np.floor(ys).astype(np.int32)
    y1 = np.minimum(y0 + 1, h - 1)
    wa = (x1 - xs) * (y1 - ys)
    wb = (xs - x0) * (y1 - ys)
    wc = (x1 - xs) * (ys - y0)
    wd = (xs - x0) * (ys - y0)
    if image.ndim == 2:
        Ia = image[y0, x0]
        Ib = image[y0, x1]
        Ic = image[y1, x0]
        Id = image[y1, x1]
        result = wa * Ia + wb * Ib + wc * Ic + wd * Id
    else:
        Ia = image[y0, x0, :]
        Ib = image[y0, x1, :]
        Ic = image[y1, x0, :]
        Id = image[y1, x1, :]
        result = (
            wa[..., None] * Ia
            + wb[..., None] * Ib
            + wc[..., None] * Ic
            + wd[..., None] * Id
        )
    return result.astype(image.dtype, copy=False)


__all__ = [
    "ARCFACE_TEMPLATE",
    "FaceQualityConfig",
    "RetinaFaceConfig",
    "FaceEmbedderConfig",
    "FaceIdentifierConfig",
    "FaceQuality",
    "FaceDetection",
    "AlignedFace",
    "RetinaFaceDetector",
    "FaceEmbedder",
    "FaceMatch",
    "FaceIdentifier",
    "align_face",
    "normalize_embedding",
]
