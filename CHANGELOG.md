# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- ROS 2 perception stack including camera, detector, tracker, event manager,
  lighting control and ROS→Django bridge nodes.
- ByteTrack-inspired tracker utilities and YOLOv8n ONNX integration.
- Lighting control rules with manual override service and light driver stub.
- Privacy-aware bridge with HTTP/WebSocket transports and offline queue.
- Launch files, sample configs, unit tests, ADR and comprehensive docs.
- GitHub Actions workflow running linting, typing, tests and doc generation.

### Changed
- Reworked the Altinet interfaces package to use the `ament_python` build type.
