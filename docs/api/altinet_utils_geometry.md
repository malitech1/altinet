# altinet.utils.geometry

Geometry helpers for projecting detections into room coordinates.

## BoundingBox

Axis-aligned bounding box expressed as x, y, width and height.

Attributes:
    x: X coordinate of the top-left corner in pixels.
    y: Y coordinate of the top-left corner in pixels.
    w: Box width in pixels.
    h: Box height in pixels.

## Path

PurePath subclass that can make system calls.

Path represents a filesystem path but unlike PurePath, also offers
methods to do system calls on path objects. Depending on your system,
instantiating a Path will return either a PosixPath or a WindowsPath
object. You can also instantiate a PosixPath or WindowsPath directly,
but cannot instantiate a WindowsPath on a POSIX system or vice versa.

## RoomGeometry

Holds calibration information for a room.

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

## load_file

Load a configuration file supporting YAML or JSON.

## load_room_geometry

Load calibration metadata for a room.

Args:
    calibration_dir: Directory containing calibration ``*.json`` files.
    room_id: Identifier of the room.

Returns:
    Loaded :class:`RoomGeometry` instance. When a calibration file is not
    found, a default configuration with ``None`` values is returned to keep
    the pipeline operational in development environments.

## normalise_centroid

Convert a bounding box centroid to normalised room coordinates.
