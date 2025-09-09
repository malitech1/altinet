# altinet

An AI assistant using computer vision to contextualise and interpret the world.

This repository currently contains a skeleton implementation. See
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for a description of the project
layout and intended components.

## Installation

The repository includes a helper script for setting up a Python virtual
environment and installing the required packages:

```bash
./install.sh
```

This creates a `.venv` directory with the dependencies listed in
`requirements.txt`. Activate the environment with:

```bash
source .venv/bin/activate
```

## ROS 2 Workspace



 
Altinet is organised as a ROS 2 workspace. Ensure you have a ROS 2
installation and the `colcon` build tool available. The computer vision
nodes also rely on OpenCV and `cv_bridge`; face recognition requires the
optional `face_recognition` Python package.

To build and run the included example nodes:
=======

Altinet is organised as a ROS 2 workspace. To build and run the included
example node:
 
 

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
=======
=======
```


 
## Local Node GUI

A minimal Django-based GUI is available for running a local Altinet node.
It now includes a simple map editor for placing nodes around your home and a
live view that reports basic environmental information.
To launch the development server:

```bash
python -m altinet.localnode.startup
```

By default the server binds to `127.0.0.1:8000`. You can override the host
and port by passing arguments to `start_server`.
