# altinet

An AI assistant using computer vision to contextualise and interpret the world.

This repository currently contains a skeleton implementation. See
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for a description of the project
layout and intended components.

## ROS 2 Workspace

Altinet is organised as a ROS 2 workspace. To build and run the included
example nodes:

```bash
colcon build
source install/setup.bash
ros2 run altinet minimal_node
# publish images from the default camera
ros2 run altinet camera_node
# detect faces in the image stream
ros2 run altinet face_detector_node
# attempt to identify detected faces
ros2 run altinet face_identifier_node
```

The vision nodes depend on OpenCV, `cv_bridge` and the optional
`face_recognition` Python package.

## Local Node GUI

A minimal Django-based GUI is available for running a local Altinet node.
To launch the development server:

```bash
python -m altinet.localnode.startup
```

By default the server binds to `127.0.0.1:8000`. You can override the host
and port by passing arguments to `start_server`.
