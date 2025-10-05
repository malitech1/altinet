# Models

Place YOLOv8n ONNX (`yolov8n.onnx`) in this directory. Download from the
[Ultralytics release page](https://github.com/ultralytics/ultralytics).
Optional ReID weights for advanced trackers can also live here.

When the ROS 2 workspace is built with `colcon`, these assets are installed to
`<install-prefix>/share/altinet/assets/models`. Copy your downloaded ONNX model
into that install-space directory (for example,
`ros2_ws/install/altinet/share/altinet/assets/models/yolov8n.onnx`) so the
runtime nodes can locate the weights after installation.
