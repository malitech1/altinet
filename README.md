# Altinet

Altinet is a home perception stack that detects, tracks and contextualises
people in real-time. The project couples a ROS 2 based pipeline with a
Django backend for dashboards and historical analysis.

## Features

- **Camera ingestion** supporting USB or RTSP sources per room.
- **Person detection** using YOLOv8n ONNX with CPU-friendly inference.
- **ByteTrack-inspired tracking** providing stable IDs across frames.
- **Event manager** deriving entry/exit/position change events and room
  presence counts.
- **Lighting control** with rule engine and manual override service.
- **ROS→Django bridge** streaming events, tracks and presence to the web
  stack with privacy-aware buffering.

## Repository layout

- `ros2_ws/src/altinet`: ROS 2 Python package with nodes, configs and tests.
- `assets/models`: ONNX models (download `yolov8n.onnx` here).
- `assets/calibration`: Room calibration files (homography, ROI, etc.).
- `docs`: Architecture notes, ADRs and generated API docs.

## Getting started

1. Install dependencies into a virtual environment:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

2. Build the ROS 2 workspace (requires ROS 2 Foxy/Humble and `colcon`):

   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

3. Launch the full stack for a single room:

   ```bash
   ros2 launch altinet altinet_full_system.launch.py room_id:=living_room
   ```

   Or run individual nodes for debugging:

   ```bash
  ros2 run altinet detector_node --ros-args -p room_id:=living_room
  ros2 run altinet tracker_node
  ros2 run altinet event_manager_node --ros-args -p presence_timeout_s:=3.0
  ros2 run altinet lighting_control_node --ros-args -p cooldown_s:=15.0
  ros2 run altinet ros2_django_bridge_node --ros-args -p privacy_config:=config/privacy.yaml
  ```

  The detector node automatically loads the packaged YOLO configuration. Override the path with ``-p config:=/custom/path.yaml`` if you need a different model setup.

4. Regenerate documentation:

   ```bash
   make docs
   ```

5. Run the offline demo (requires a recorded video):

   ```bash
   python scripts/run_demo.py path/to/video.mp4 --room living_room
   ```

## Identity service walkthrough

The identity service classifies each detection as a resident (`user`),
`guest` or `unknown` based on the bounding box size and the detector's
confidence score. Follow the steps below to configure and exercise it.

### 1. Start the identity node

- Launching the full stack via `altinet_full_system.launch.py` already brings
  up `identity_node` alongside the detector.
- To run it independently (useful for testing parameters) execute:

  ```bash
  ros2 run altinet identity_node \
    --ros-args \
    -p user_min_area_ratio:=0.02 \
    -p min_detection_confidence:=0.3
  ```

  The node advertises `/altinet/check_person_identity` once the service is
  ready.

### 2. Tune classification thresholds

`identity_node` exposes the following ROS parameters:

- `user_min_area_ratio` – minimum person-to-frame area ratio to treat someone
  as a resident.
- `min_detection_confidence` – detections below this score are labelled
  `unknown`.
- `user_confidence_bonus` – extra confidence awarded when the resident
  threshold is met.
- `guest_confidence_scale` and `unknown_confidence_scale` – multipliers that
  temper the returned confidence for guests or low-confidence detections.

Persist overrides in a YAML file if you want them applied whenever the service
starts:

```yaml
identity_node:
  ros__parameters:
    user_min_area_ratio: 0.018
    guest_confidence_scale: 0.55
```

Pass the file to the node with `--params-file path/to/identity.yaml` during
launch.

### 3. Exercise the service manually

Use the ROS 2 CLI to send a sample request and inspect the response:

```bash
ros2 service call /altinet/check_person_identity \
  altinet_interfaces/srv/CheckPersonIdentity "{track_id: 7, x: 320.0, y: 180.0, \
  w: 220.0, h: 360.0, detection_confidence: 0.82, room_id: 'living_room', \
  frame_id: 'camera_link', image_width: 1280, image_height: 720}"
```

The service returns a label (`user`, `guest` or `unknown`), whether the person
is a user, a confidence score and a human-readable reason. You can rerun the
command after tweaking parameters to see how the classification changes.

### 4. Integrate with the detector

`detector_node` calls the service asynchronously whenever
`identity_service_enabled` is `true` (the default). It logs the resolved
identity beside the bounding box coordinates. Disable the integration or adjust
its service wait timeout with:

```bash
ros2 param set /detector_node identity_service_enabled false
ros2 param set /detector_node identity_service_timeout 0.5
```

Re-enable the service or lower the timeout when you are ready to resume
classification.

## Testing

Unit tests run without ROS message generation thanks to pure-Python
components:

```bash
pytest
```

## Privacy

By default `config/privacy.yaml` disables upstream forwarding. Set
`forward_allowed: true` when you are ready to stream events to the Django
backend.
