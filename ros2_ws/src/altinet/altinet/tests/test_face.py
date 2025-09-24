"""Tests for the face recognition utilities."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from altinet.utils.face import (
    ARCFACE_TEMPLATE,
    FaceEmbedder,
    FaceEmbedderConfig,
    FaceIdentifier,
    FaceIdentifierConfig,
    align_face,
    normalize_embedding,
)


def test_align_face_maps_landmarks_to_template():
    image = np.zeros((160, 160, 3), dtype=np.uint8)
    angle = np.deg2rad(20.0)
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]],
        dtype=np.float32,
    )
    scale = 1.25
    offset = np.array([14.0, -18.0], dtype=np.float32)
    landmarks = ARCFACE_TEMPLATE @ rotation.T * scale + offset

    aligned = align_face(image, landmarks)

    homogeneous = np.hstack([landmarks, np.ones((5, 1), dtype=np.float32)])
    transformed = (aligned.transform @ homogeneous.T).T[:, :2]

    assert np.allclose(transformed, ARCFACE_TEMPLATE, atol=1e-1)


class DummySession:
    """Simple ONNXRuntime stub returning a predefined embedding."""

    def __init__(self, output: np.ndarray) -> None:
        self._inputs = [SimpleNamespace(name="input")]
        self._output = output

    def get_inputs(self):  # pragma: no cover - trivial wrapper
        return self._inputs

    def run(self, _outputs, _feeds):
        return [self._output]


def test_face_embedder_normalises_embeddings():
    config = FaceEmbedderConfig(model_path=Path("embedder.onnx"), embedding_dim=4)
    session = DummySession(np.array([[2.0, 0.0, 0.0, 0.0]], dtype=np.float32))
    embedder = FaceEmbedder(config, session=session)
    dummy_face = np.full((112, 112, 3), 127.5, dtype=np.uint8)

    embedding = embedder.embed(dummy_face)

    assert embedding.shape == (4,)
    assert np.linalg.norm(embedding) == pytest.approx(1.0)


class FakeFaissIndex:
    """Minimal FAISS-like index used for tests."""

    def __init__(self, dim: int) -> None:
        self.dim = dim
        self.vectors: list[np.ndarray] = []

    def add(self, matrix: np.ndarray) -> None:
        for row in matrix:
            self.vectors.append(np.asarray(row, dtype=np.float32))

    def search(self, matrix: np.ndarray, top_k: int):
        query = np.asarray(matrix[0], dtype=np.float32)
        if not self.vectors:
            scores = -np.ones((1, top_k), dtype=np.float32)
            indices = -np.ones((1, top_k), dtype=np.int64)
            return scores, indices
        sims = np.array([float(np.dot(query, vec)) for vec in self.vectors], dtype=np.float32)
        order = np.argsort(-sims)
        top_indices = order[:top_k]
        top_scores = sims[top_indices]
        if top_scores.shape[0] < top_k:
            pad_scores = np.full(top_k - top_scores.shape[0], -1.0, dtype=np.float32)
            pad_indices = np.full(top_k - top_indices.shape[0], -1, dtype=np.int64)
            top_scores = np.concatenate([top_scores, pad_scores])
            top_indices = np.concatenate([top_indices, pad_indices])
        return top_scores.reshape(1, -1), top_indices.reshape(1, -1)


def test_face_identifier_respects_similarity_threshold():
    config = FaceIdentifierConfig(embedding_dim=4, similarity_threshold=0.5, top_k=2)
    index = FakeFaissIndex(dim=4)
    identifier = FaceIdentifier(config, index=index)
    identifier.add("alice", normalize_embedding(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)))
    identifier.add("bob", normalize_embedding(np.array([0.8, 0.2, 0.0, 0.0], dtype=np.float32)))

    match = identifier.identify(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
    assert match is not None
    assert match.label == "alice"
    assert match.score == pytest.approx(1.0)

    miss = identifier.identify(np.array([-1.0, 0.0, 0.0, 0.0], dtype=np.float32))
    assert miss is None
