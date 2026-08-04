"""
Reachy 2 - shape prior for object dimension estimation.

Grasp/pre-grasp pose planning used to live in this file; it has been removed
to be redesigned from scratch. What's left is only the YOLO-class -> shape
prior that bomi_detection._object_dimensions needs to decide whether to use
the cylinder circle fit for width_m.
"""

# Flat/rectangular items map to "box"; blocky/roughly-cubic ones to "cube"
# (geometrically identical here, kept distinct only for readability).
SHAPE_BY_CLASS = {
    "bottle": "cylinder", "cup": "cylinder", "wine glass": "cylinder", "vase": "cylinder",
    "carrot": "cylinder", "hair drier": "cylinder", "toothbrush": "cylinder",
    "spoon": "cylinder", "fork": "cylinder", "knife": "cylinder", "banana": "cylinder",
    "apple": "sphere", "orange": "sphere", "donut": "sphere",
    "book": "box", "cell phone": "box", "remote": "box", "laptop": "box",
    "keyboard": "box", "tv": "box", "sandwich": "box",
    "mouse": "cube", "bowl": "cube", "cake": "cube", "teddy bear": "cube",
    "broccoli": "cube", "scissors": "cube",
}
DEFAULT_SHAPE = "cylinder"


def shape_from_class(class_name):
    """Rough shape prior from the YOLO class name; defaults to cylinder for
    anything not in the table."""
    return SHAPE_BY_CLASS.get(class_name, DEFAULT_SHAPE)
