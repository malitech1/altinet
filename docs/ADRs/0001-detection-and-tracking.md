# ADR 0001: Detection and Tracking Stack

## Context

Altinet needs a reliable way to detect and track people across multiple
rooms using CPU-friendly inference. The prior face-centric prototype no
longer met the requirements for room-level presence detection or ROS 2
integration.

## Decision

We implemented a YOLOv8n ONNX based detector paired with a
ByteTrack-inspired tracker. The detector runs with onnxruntime and only
reports the person class. Tracking uses IoU matching with a configurable
max age to preserve identities across occlusions.

## Alternatives

- Use YOLOv5s or TensorRT for inference. Rejected to keep the dependency
  footprint light and maintain CPU compatibility.
- DeepSORT for tracking. Rejected due to the additional re-identification
  model and increased runtime cost.

## Consequences

- The pipeline runs fully on CPU and provides stable identities for the
  event manager.
- ByteTrack behaviour is approximated; long occlusions may still break
  identities, but the modular design allows swapping in a more advanced
  implementation later.

## Links

- `ros2_ws/src/altinet/altinet/utils/models.py`
- `ros2_ws/src/altinet/altinet/utils/tracking.py`
