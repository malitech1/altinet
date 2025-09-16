# altinet.nodes.detector_node

Detector node performing YOLOv8 inference.

## Detection

Represents a single person detection.

Attributes:
    bbox: Bounding box of the detection in pixel coordinates.
    confidence: Confidence score assigned by the detector.
    room_id: Room identifier associated with the source camera.
    frame_id: Identifier of the originating sensor frame.
    timestamp: UTC timestamp indicating when the frame was captured.
    image_size: Original image dimensions as ``(height, width)``.

## DetectorNode

ROS 2 node wrapping :class:`DetectorPipeline`.

## DetectorPipeline

High level interface around :class:`YoloV8Detector`.

## Node

The base class of the class hierarchy.

When called, it accepts no arguments and returns a new featureless
instance that has no instance attributes and cannot be given any.

## Path

PurePath subclass that can make system calls.

Path represents a filesystem path but unlike PurePath, also offers
methods to do system calls on path objects. Depending on your system,
instantiating a Path will return either a PosixPath or a WindowsPath
object. You can also instantiate a PosixPath or WindowsPath directly,
but cannot instantiate a WindowsPath on a POSIX system or vice versa.

## YoloConfig

Configuration for the YOLOv8 ONNX detector.

## YoloV8Detector

Run YOLOv8 ONNX inference for person detection.

## datetime

datetime(year, month, day[, hour[, minute[, second[, microsecond[,tzinfo]]]]])

The year, month and day arguments are required. tzinfo may be None, or an
instance of a tzinfo subclass. The remaining arguments may be ints.

## deque

deque([iterable[, maxlen]]) --> deque object

A list-like sequence optimized for data accesses near its endpoints.

## detections_to_msg

Convert detections to a ROS message.

## load_config

Load YOLO configuration from YAML.

## load_file

Load a configuration file supporting YAML or JSON.

## main
