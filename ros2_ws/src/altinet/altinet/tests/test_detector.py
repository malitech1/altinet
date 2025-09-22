"""Unit tests for the detector pipeline."""

from datetime import datetime
from pathlib import Path

import numpy as np

from altinet.nodes.detector_node import DetectorPipeline, load_config
from altinet.utils.models import _scale_box
from altinet.utils.types import BoundingBox, Detection


class DummyDetector:
    """Simple detector returning a preset detection."""

    def __init__(self, detection: Detection) -> None:
        self.detection = detection
        self.calls = 0

    def detect(self, frame, room_id, frame_id, timestamp):
        self.calls += 1
        return [self.detection]


def test_detector_pipeline_returns_detections(tmp_path):
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    detection = Detection(
        bbox=BoundingBox(10.0, 20.0, 50.0, 100.0),
        confidence=0.9,
        room_id="living_room",
        frame_id="cam",
        timestamp=datetime.utcnow(),
        image_size=frame.shape[:2],
    )
    pipeline = DetectorPipeline(detector=DummyDetector(detection))
    results = pipeline.process(frame, "living_room", "cam", datetime.utcnow())
    assert results == [detection]
    assert pipeline.fps == 0.0 or pipeline.fps > 0.0


def test_load_config_reads_yaml(tmp_path):
    config_path = tmp_path / "yolo.yaml"
    config_path.write_text(
        "model_path: assets/models/yolov8n.onnx\nconf_thresh: 0.5\n", encoding="utf8"
    )
    config = load_config(config_path)
    assert config.model_path == Path("assets/models/yolov8n.onnx")
    assert config.conf_thresh == 0.5


def test_scale_box_clamps_boxes_outside_top_left():
    box = np.array([-30.0, -20.0, 40.0, 60.0], dtype=np.float32)
    scaled = _scale_box(box, ratio=1.0, padding=(0.0, 0.0), original_shape=(100, 200))
    bbox = BoundingBox(*scaled)

    assert bbox.x >= 0.0
    assert bbox.y >= 0.0
    assert bbox.w >= 0.0
    assert bbox.h >= 0.0
    assert bbox.w == 0.0
