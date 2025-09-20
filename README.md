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
