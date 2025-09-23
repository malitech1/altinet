# altinet.nodes.visualizer_node

ROS 2 node that overlays the latest detections and tracks on camera images
for RViz or republished topics.

## Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `timestamp_tolerance` | float seconds | `1.0` | Maximum age difference allowed between the camera frame and the associated detections/tracks before they are considered stale. |
| `stale_grace_period` | float seconds | `2.0` | Additional window after `timestamp_tolerance` during which the previous results remain visible using a muted style so operators can bridge brief upstream delays. Values below the tolerance collapse to the tolerance to preserve historical behaviour. |
| `frame_process_interval` | float seconds | `0.5` | Minimum duration between redraws. Lower the interval for quicker refreshes at the expense of more CPU load, or raise it to conserve resources on slower machines. Set to a non-positive value to process every incoming frame. |
| `draw_detections` | bool | `true` | Enable drawing raw detections (used when no track information is available). |
| `draw_tracks` | bool | `true` | Enable drawing tracker outputs when available. |
| `display_window` | bool | `true` | Show the annotated image in a local OpenCV window. |

The interplay between `timestamp_tolerance` and `stale_grace_period` controls
responsiveness: smaller tolerances remove outdated overlays quickly, whereas
larger grace periods keep boxes on-screen during short processing gaps while
softening their colours to signal that the data is ageing.

