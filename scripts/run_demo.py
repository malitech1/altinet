"""Run an end-to-end demo without ROS by processing a recorded video."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import cv2

from altinet.nodes.detector_node import DetectorPipeline, load_config
from altinet.nodes.tracker_node import TrackerPipeline
from altinet.utils.config import default_yolo_config_path
from altinet.utils.models import YoloV8Detector


def run_demo(video_path: Path, config_path: Path, room_id: str) -> None:
    config = load_config(config_path)
    detector = DetectorPipeline(YoloV8Detector(config))
    tracker = TrackerPipeline()
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Unable to open video: {video_path}")

    frame_id = "demo_camera"
    while True:
        success, frame = capture.read()
        if not success:
            break
        timestamp = datetime.utcnow()
        detections = detector.process(frame, room_id, frame_id, timestamp)
        tracks = tracker.update(detections)
        print(f"Frame @ {timestamp.isoformat()} - {len(tracks)} tracks")
    capture.release()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path, help="Path to a demo video file")
    parser.add_argument(
        "--config",
        type=Path,
        default=default_yolo_config_path(),
        help="YOLO config path",
    )
    parser.add_argument("--room", default="demo_room", help="Room identifier")
    args = parser.parse_args()
    run_demo(args.video, args.config, args.room)


if __name__ == "__main__":
    main()
