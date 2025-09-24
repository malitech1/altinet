# Altinet Perception Stack

## What changed

- Replaced the face-oriented prototype with a room-centric perception
  stack built on ROS 2 nodes.
- Added YOLOv8n ONNX inference, ByteTrack-based tracking, event
  management, lighting control, and a ROS→Django bridge.
- Documented the architecture and generated API docs via `pdoc`.
- Added a persisted face gallery with FAISS-compatible exports, enrolment
  audit trails and REST APIs for managing identities.

## Pipeline overview

```mermaid
graph LR
    A[Camera Node] --> B[Detector Node]
    B --> C[Tracker Node]
    B --> G[Identity Service]
    C --> D[Event Manager]
    D --> E[Lighting Control]
    C --> F[ROS→Django Bridge]
    D --> F
```

### Detector

`DetectorNode` loads YOLOv8n from `assets/models/yolov8n.onnx`, filters to
person detections, and publishes `altinet/PersonDetections`. It exposes a
`min_detection_interval` parameter (seconds) that throttles how often new
frames are pushed through the network. The default is now `1.5` seconds to
significantly reduce inference load. Lower the interval if your hardware can
handle additional work to tighten responsiveness.

Example parameter override:

```yaml
detector_node:
  ros__parameters:
    min_detection_interval: 0.25  # override default 1.5s cadence
```

You can also adjust the parameter at runtime:

```bash
ros2 param set /detector_node min_detection_interval 0.25
```

Tune the interval based on hardware characteristics—the higher the value, the
more frames are skipped, reducing workload at the cost of responsiveness.

### Identity Service

`IdentityNode` exposes `/altinet/check_person_identity`, using bounding box
size and detector confidence to distinguish residents from guests. The
detector queries the service asynchronously for each detection and logs the
resolved identity alongside the bounding box coordinates.

### Face recognition gallery

`FaceRecognitionService` maintains a FAISS-friendly embedding matrix on disk
and writes JSON snapshots whenever a new enrolment is attempted. The ROS 2
`face_enroller` service accepts uploads from the web UI and notifies running
nodes about updates. Django exposes `/api/identities/`, `/api/face-embeddings/`
and `/api/face-snapshots/` so that operators can review audit trails and
trigger new enrolments directly from the dashboard. The ROS→Django bridge
forwards `FaceSnapshot` events and enrolment confirmations using the same
privacy-aware backoff logic as room events.

### Tracker

`TrackerNode` wraps a ByteTrack-inspired tracker that maintains stable
IDs. It publishes `altinet/PersonTracks` for downstream consumers and now
periodically extrapolates bounding boxes between detector updates using the
tracked velocity. This keeps the visualizer aligned with the person's motion
even while inference is throttled to every 1.5 seconds.

### Event Manager

The event manager derives ENTRY/EXIT/POSITION_CHANGE events and publishes
room presence summaries. Calibration is loaded from
`assets/calibration/<room>.json` when available.

### Lighting Control

Lighting rules toggle rooms based on occupancy and support manual
overrides via `/altinet/manual_light_override`.

### ROS→Django Bridge

The bridge forwards events, presence and track updates to the Django
backend using WebSocket when available, falling back to REST with
exponential backoff.

## Running docs

Use `make docs` to regenerate API documentation under `docs/api/`.
