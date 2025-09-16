# altinet.nodes.ros2_django_bridge_node

Bridge node forwarding ROS data to the Django backend.

## BoundingBox

Axis-aligned bounding box expressed as x, y, width and height.

Attributes:
    x: X coordinate of the top-left corner in pixels.
    y: Y coordinate of the top-left corner in pixels.
    w: Box width in pixels.
    h: Box height in pixels.

## BridgeConfig

Configuration for the ROS → Django bridge.

## BridgeTransport

Abstract transport for delivering payloads.

## DjangoBridge

Buffers and forwards events to the Django backend.

## EventModel

Represents a semantic event emitted by the event manager.

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

## RequestsTransport

Sends payloads using HTTP POST requests.

## RoomPresenceModel

Presence summary for a room.

## Ros2DjangoBridgeNode

ROS 2 node that forwards events and presence updates to Django.

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

## WebSocketTransport

Sends payloads over a persistent WebSocket connection.

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

## datetime

datetime(year, month, day[, hour[, minute[, second[, microsecond[,tzinfo]]]]])

The year, month and day arguments are required. tzinfo may be None, or an
instance of a tzinfo subclass. The remaining arguments may be ints.

## deque

deque([iterable[, maxlen]]) --> deque object

A list-like sequence optimized for data accesses near its endpoints.

## load_file

Load a configuration file supporting YAML or JSON.

## load_privacy_config

## main
