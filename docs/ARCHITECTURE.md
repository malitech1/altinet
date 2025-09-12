# Altinet Architecture

This repository contains the initial skeleton for the Altinet system. The structure is intended to
serve as a foundation for future development and includes placeholders for core modules and
services.

```
altinet/
├── docs/                  # Project documentation
├── src/
│   └── altinet/           # ROS 2 package
│       ├── package.xml    # Package manifest
│       ├── setup.py       # ament package setup
│       └── altinet/       # Application source code
│           ├── core/      # Core networking components
│           ├── services/  # High level services built on the core
│           ├── utils/     # Shared utilities and helpers
│           ├── localnode/ # Local Django node
│           └── nodes/     # ROS 2 nodes
└── tests/                 # Test suite
```

## Components

- **core**: Low level networking primitives that power Altinet.
- **services**: Higher level capabilities such as discovery and messaging built on top of the core.
- **utils**: Shared helpers including cryptographic utilities.
- **localnode**: A minimal Django application for running a local Altinet node
  with a map editor and live view.
- **nodes**: ROS 2 nodes that expose Altinet functionality at runtime.
- **localnode**: A minimal Django application for running a local Altinet node.
- **nodes**: ROS 2 nodes that expose Altinet functionality at runtime.
  - `minimal_node`: placeholder node that logs a startup message.
  - `camera_node`: publishes images from a camera device.
  - `face_detector_node`: detects faces and publishes bounding boxes after a time-based confidence check.
  - `face_identifier_node`: attempts to match faces against known identities.
- **tests**: Unit tests that exercise the behavior of the system.

Each module is currently a placeholder and should be expanded as the project grows.
