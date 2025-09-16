# altinet.nodes.tracker_node

Tracker node maintaining persistent person identities.

## BoundingBox

Axis-aligned bounding box expressed as x, y, width and height.

Attributes:
    x: X coordinate of the top-left corner in pixels.
    y: Y coordinate of the top-left corner in pixels.
    w: Box width in pixels.
    h: Box height in pixels.

## ByteTrack

A lightweight implementation of the ByteTrack algorithm.

## ByteTrackConfig

Configuration for the multi-object tracker.

## Detection

Represents a single person detection.

Attributes:
    bbox: Bounding box of the detection in pixel coordinates.
    confidence: Confidence score assigned by the detector.
    room_id: Room identifier associated with the source camera.
    frame_id: Identifier of the originating sensor frame.
    timestamp: UTC timestamp indicating when the frame was captured.
    image_size: Original image dimensions as ``(height, width)``.

## Node

The base class of the class hierarchy.

When called, it accepts no arguments and returns a new featureless
instance that has no instance attributes and cannot be given any.

## Track

Represents a tracked person across frames.

Attributes:
    track_id: Unique identifier for the track.
    bbox: Current bounding box of the tracked person.
    confidence: Confidence score inherited from the last detection.
    room_id: Room identifier associated with the track.
    timestamp: Timestamp of the most recent update.
    image_size: Dimensions of the frame that produced the update.
    velocity: Estimated pixel displacement between consecutive frames.
    hits: Number of successful detection associations.
    age: Number of consecutive updates without a detection match.

## TrackerNode

ROS 2 node bridging detection and tracking stages.

## TrackerPipeline

Wraps :class:`ByteTrack` to provide a simplified interface.

## datetime

datetime(year, month, day[, hour[, minute[, second[, microsecond[,tzinfo]]]]])

The year, month and day arguments are required. tzinfo may be None, or an
instance of a tzinfo subclass. The remaining arguments may be ints.

## main

## tracks_to_msg
