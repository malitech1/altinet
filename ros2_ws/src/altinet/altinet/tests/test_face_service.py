from __future__ import annotations

from pathlib import Path

import numpy as np

from altinet.services.face_recognition import FaceRecognitionService


def test_enrolment_writes_gallery(tmp_path):
    gallery_dir = tmp_path / "gallery"
    service = FaceRecognitionService(gallery_dir)
    calls = []

    service.register_listener(lambda result: calls.append(result.identity_id))

    high_quality = np.tile(np.linspace(0, 1, 128, dtype=np.float32), (64, 1))
    low_quality = np.zeros((64, 64), dtype=np.float32)
    samples = [
        (high_quality, {"source": "a.jpg"}),
        (low_quality, {"source": "b.jpg"}),
    ]
    result = service.enrol_samples(
        "alice",
        samples,
        quality_threshold=0.1,
        track_id=7,
        camera_id="cam-1",
        extra_metadata={"session": "unit-test"},
    )

    assert result.accepted_count == 1
    assert result.rejected_count == 1
    assert calls == ["alice"]

    index_path = gallery_dir / "gallery.npy"
    faiss_path = gallery_dir / "gallery.faiss"
    metadata_path = gallery_dir / "metadata.json"
    snapshots_dir = gallery_dir / "snapshots"

    assert index_path.exists()
    assert faiss_path.exists()
    assert metadata_path.exists()
    snapshots = list(snapshots_dir.glob("*.json"))
    assert len(snapshots) == 1

    matrix = np.load(index_path)
    assert matrix.shape[0] == 1

    metadata = metadata_path.read_text(encoding="utf-8")
    assert "alice" in metadata

    result2 = service.enrol_embeddings(
        "bob",
        [([0.2] * service.embedding_dim, {"quality": 0.8})],
        camera_id="cam-1",
    )
    assert result2.accepted_count == 1
    matrix = np.load(index_path)
    assert matrix.shape[0] == 2
