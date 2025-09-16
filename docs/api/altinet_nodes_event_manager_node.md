# altinet.nodes.event_manager_node

Event manager node for generating semantic events.

## BoundingBox

Axis-aligned bounding box expressed as x, y, width and height.

Attributes:
    x: X coordinate of the top-left corner in pixels.
    y: Y coordinate of the top-left corner in pixels.
    w: Box width in pixels.
    h: Box height in pixels.

## EventManager

Processes tracks and emits semantic events and presence summaries.

## EventManagerConfig

Configuration options for :class:`EventManager`.

## EventManagerNode

ROS 2 wrapper around :class:`EventManager`.

## EventModel

Represents a semantic event emitted by the event manager.

## FloorplanAdapter

Load and provide access to floorplan metadata.

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

## RoomGeometry

Holds calibration information for a room.

## RoomPresenceModel

Presence summary for a room.

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

## datetime

datetime(year, month, day[, hour[, minute[, second[, microsecond[,tzinfo]]]]])

The year, month and day arguments are required. tzinfo may be None, or an
instance of a tzinfo subclass. The remaining arguments may be ints.

## defaultdict

defaultdict(default_factory=None, /, [...]) --> dict with default factory

The default factory is called without arguments to produce
a new value when a key is not present, in __getitem__ only.
A defaultdict compares equal to a dict with the same items.
All remaining arguments are treated the same as if they were
passed to the dict constructor, including keyword arguments.

## event_to_msg

## load_room_geometry

Load calibration metadata for a room.

Args:
    calibration_dir: Directory containing calibration ``*.json`` files.
    room_id: Identifier of the room.

Returns:
    Loaded :class:`RoomGeometry` instance. When a calibration file is not
    found, a default configuration with ``None`` values is returned to keep
    the pipeline operational in development environments.

## main

## normalise_centroid

Convert a bounding box centroid to normalised room coordinates.

## presence_to_msg

## timedelta

Difference between two datetime values.

timedelta(days=0, seconds=0, microseconds=0, milliseconds=0, minutes=0, hours=0, weeks=0)

All arguments are optional and default to 0.
Arguments may be integers or floats, and may be positive or negative.
