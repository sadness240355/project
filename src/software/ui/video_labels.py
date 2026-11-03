"""Labels for the deployed engine; unknown classes remain explicit numeric IDs."""
import json
from pathlib import Path
try:
    NAMES = json.loads((Path(__file__).parent / "video_class_names.json").read_text())["names"]
except (OSError, ValueError, KeyError):
    NAMES = {}

def box_label(box):
    class_id, track_id = int(box[5]), int(box[6])
    name = NAMES.get(str(class_id), "class:%d" % class_id)
    identity = str(track_id) if track_id >= 0 else "--"
    return "%s ID:%s %.2f" % (name, identity, box[4])
