# altinet.utils.tracking

Tracking utilities implementing a ByteTrack-inspired tracker.

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

## dataclass

Add dunder methods based on the fields defined in the class.

Examines PEP 526 __annotations__ to determine fields.

If init is true, an __init__() method is added to the class. If repr
is true, a __repr__() method is added. If order is true, rich
comparison dunder methods are added. If unsafe_hash is true, a
__hash__() method is added. If frozen is true, fields may not be
assigned to after instance creation. If match_args is true, the
__match_args__ tuple is added. If kw_only is true, then by default
all fields are keyword-only. If slots is true, a new class with a
__slots__ attribute is returned.
