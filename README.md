![Altinet logo](assets/branding/Altinet%20Logo.png)

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
- **Face recognition gallery** with FAISS exports, audit snapshots and
  enrolment APIs wired to the Django backend.
- **ROS→Django bridge** streaming events, tracks, presence and face
  snapshots to the web
  stack with privacy-aware buffering.
- **Bootstrap web dashboard** served directly from Django with login and
  refreshed admin tooling.

## Repository layout

- `ros2_ws/src/altinet`: ROS 2 Python package with nodes, configs and tests.
- `assets/models`: ONNX models (download `yolov8n.onnx` here).
- `assets/calibration`: Room calibration files (homography, ROI, etc.).
- `docs`: Architecture notes, ADRs and generated API docs.
- `backend/web`: Lightweight Django views, templates and tests for the UI.

## Getting started

1. Install dependencies into a virtual environment:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

2. Prepare the Django database and start the web interface:

   ```bash
   python backend/manage.py migrate
   python backend/manage.py createsuperuser  # optional but recommended
   python backend/manage.py runserver
   ```

   The UI is served from Django and is available at
   http://127.0.0.1:8000/ with the admin at http://127.0.0.1:8000/admin/.

3. Build the ROS 2 workspace (requires ROS 2 Foxy/Humble and `colcon`):

   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

4. Launch the full stack for a single room:

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

5. Regenerate documentation:

   ```bash
   make docs
   ```

6. Run the offline demo (requires a recorded video):

   ```bash
 python scripts/run_demo.py path/to/video.mp4 --room living_room
  ```

## Query the built-in LLM assistant

Altinet exposes a lightweight `/api/llm/` namespace in Django that wraps the
offline LLaMA model configured through environment variables. To enable the
assistant locally:

1. Point the backend at your `llama.cpp` compatible weights and optional
   generation settings before starting Django:

   ```bash
   export ALTINET_LLAMA_MODEL_PATH=/absolute/path/to/ggml-model.bin
   export ALTINET_LLAMA_CONTEXT=2048            # optional
   export ALTINET_LLAMA_MAX_TOKENS=256          # optional
   export ALTINET_LLAMA_TEMPERATURE=0.7         # optional
   ```

2. Verify the model loads correctly by calling the health probe:

   ```bash
   curl http://127.0.0.1:8000/api/llm/health/
   ```

3. Send prompts with your preferred HTTP client. The service accepts JSON with
   a `prompt` field plus optional `max_tokens` and `temperature` overrides:

   ```bash
   curl -X POST http://127.0.0.1:8000/api/llm/prompt/ \
     -H 'Content-Type: application/json' \
     -d '{
           "prompt": "Summarise the latest living_room activity feed",
           "max_tokens": 128,
           "temperature": 0.6
         }'
   ```

   Successful requests return a JSON document similar to

   ```json
   {"response": "The living room has been idle for 5 minutes."}
   ```

   The home dashboard ships with a basic form that posts to the same endpoint
   (`/` → *Assistant* card) so operators can experiment without leaving the UI.

### Query the assistant from ROS 2

The ROS 2 workspace also exposes the assistant through a synchronous service so
nodes can obtain completions without touching HTTP clients directly. After
building and sourcing `ros2_ws`, launch the bridge that proxies calls to the
Django backend:

```bash
ros2 run altinet llm_service_node \
  --ros-args \
  -p api_base:=http://127.0.0.1:8000/api/llm \
  -p default_max_tokens:=256 \
  -p default_temperature:=0.7
```

Once the node is running you should see `/altinet/llm/prompt` listed under
`ros2 service list`. Invoke it from any terminal to fetch a response:

```bash
ros2 service call /altinet/llm/prompt altinet_interfaces/srv/PromptLLM "{
  prompt: 'Summarise the latest living_room activity feed',
  max_tokens: 128,
  temperature: 0.6
}"
```

The service returns the generated `response` text together with bookkeeping
fields (`model`, `prompt_tokens`, `completion_tokens`). Leave `max_tokens` or
`temperature` unset to fall back to the defaults configured on the node. When
invoking the service from your own node import the generated interface and call
it through a `Client<PromptLLM>` just like any other ROS 2 service.

### Builder → Blender → Dashboard workflow

The browser-based builder now supports multi-level plans, local persistence and
a one-click export for Blender. Follow the detailed guide in
[`docs/workflows/floorplan_pipeline.md`](docs/workflows/floorplan_pipeline.md)
to go from a sketched plan to the OBJ that powers the 3D viewer on the home page.

### Production deployments

The Django backend defaults to debug mode so local `runserver` instances serve
static assets without extra configuration. When deploying to production
environments you should explicitly disable debug behaviour by setting
`DJANGO_DEBUG=false` in the deployment's environment configuration.

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

## Face recognition gallery

Altinet ships a lightweight face gallery manager backed by NumPy and
FAISS-compatible exports. The gallery is shared between the ROS 2 nodes,
the Django API and the enrolment CLI. It writes `gallery.npy` and
`gallery.faiss` into `assets/face_gallery/` alongside JSON snapshots that
record which curated images were accepted or rejected.

### 1. Prepare the embedding model

The service expects an ONNX embedding model (e.g. ArcFace) in
`assets/models/`. Download the model that matches your hardware and set the
path in your ROS launch configuration or environment.

```
mkdir -p assets/models
curl -L -o assets/models/arcface.onnx https://example.com/arcface.onnx
```

### 2. Tune quality thresholds

Enrolment applies a simple contrast-based quality score in the
`[0, 1]` range. Adjust the minimum quality accepted during CLI runs or via the
web UI to balance recall and false positives:

```
python scripts/add_user.py --quality-threshold 0.25 --gallery-dir assets/face_gallery
```

Snapshots that fall below the threshold are recorded in the audit log but are
not added to the FAISS index.

### 3. Run the CLI enrolment workflow

`scripts/add_user.py` now calls `FaceRecognitionService` directly. The script
captures photos, writes user metadata to `assets/users/<name>/metadata.json`
and updates the gallery directory. Accepted images are added to the index and
a JSON snapshot is stored under `assets/face_gallery/snapshots/` describing the
decision.

### 4. Manage enrolments from the web UI

The Django application exposes `/api/identities/`, `/api/face-embeddings/` and
`/api/face-snapshots/` endpoints. Creating an embedding publishes the vector to
ROS via the `/altinet/face_enroller` service and records the payload in the
database atomically. Face snapshots arriving from ROS are persisted together
with camera, track and quality metadata for audit trails.

### 5. ROS → Django forwarding

`ros2_django_bridge_node` now listens for `FaceSnapshot` and enrolment
confirmation messages. It forwards them according to the privacy configuration
and reuses the same exponential backoff logic as the other topics. Nodes that
subscribe to the `/altinet/face_gallery_updates` topic receive real-time
notifications whenever new embeddings land.

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
