# Altinet Architecture

The repository is structured around a ROS 2 workspace hosting the
perception stack and a Django web application for dashboards.

```
altinet/
├── ros2_ws/
│   └── src/altinet/          # ROS 2 Python package
│       ├── altinet/
│       │   ├── nodes/        # ROS nodes (camera, detector, tracker, etc.)
│       │   ├── utils/        # Geometry, models, tracking helpers
│       │   ├── dashboards/   # Floorplan adapter utilities
│       │   ├── config/       # YAML configuration samples
│       │   ├── launch/       # Launch files
│       │   ├── drivers/      # Hardware abstractions (lights)
│       │   ├── msgs/, srv/   # ROS interface definitions
│       │   └── tests/        # Pytest-based test suite
│       ├── package.xml       # ROS package manifest
│       └── setup.py          # ament_python entry point
├── assets/
│   ├── models/               # ONNX models
│   └── calibration/          # Room calibration JSON/YAML
├── docs/                     # Documentation, ADRs, API references
└── web/                      # Django application (to be expanded)
```

Each ROS node exposes a pure-Python core component for unit testing and a
thin ROS wrapper. Messages are defined in the `msgs/` directory with
matching dataclasses used during tests when ROS interfaces are
unavailable.

Use `colcon build` inside `ros2_ws` to build the package when running on
a ROS-enabled machine. During development the Python modules can be
loaded directly without compiling messages, though full runtime requires
ROS message generation.
