"""Per-exposure coverage admission; chassis motion is never a veto."""
import math
from .vision_geometry import FloorCamera


def exposure_view(data, now, mission_stamp, epoch, epoch_started_stamp, map_frame='map'):
    """Return timestamp, actual capture pose and that image's camera model.

    No latest-pose, stationary-window or invented zero-motion fallback exists.
    Stamp bounds reject delayed frames crossing a mission/search restart even
    when the camera publisher does not carry a mission-specific epoch field.
    """
    try:
        stamp = float(data['stamp'])
        if (data.get('ready') is not True or data.get('geometry_ready') is False
                or not all(math.isfinite(v) for v in (stamp, now, mission_stamp, epoch_started_stamp))
                or not 0. <= now-stamp <= 1.5
                or stamp < max(mission_stamp, epoch_started_stamp)
                or ('mission_stamp' in data and data['mission_stamp'] != mission_stamp)
                or ('search_epoch' in data and data['search_epoch'] != epoch)):
            return None
        capture = data['capture_pose']
        if (capture['frame_id'] != map_frame
                or abs(float(capture['stamp'])-stamp) > 1e-6):
            return None
        pose = tuple(float(capture[k]) for k in ('x', 'y', 'yaw'))
        if not all(math.isfinite(v) for v in (*pose, float(capture['stamp']))):
            return None
        if not isinstance(data.get('camera_model'),dict):return None
        camera = FloorCamera.from_metadata(data['camera_model'])
        return stamp, pose, camera
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
