#!/usr/bin/env python

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

class Orientations:
    #x, y, z define axis, w defines how much rotation around that axis
    FORWARD_POINT = Quaternion(
        x=0,
        y=0.707,
        z=0,
        w=0.707,
    )

    DOWNWARD_POINT = Quaternion(
        x=1,
        y=0,
        z=0,
        w=0,
    )

    LEFTWARD_POINT = Quaternion(
        x=0.707,
        y=0,
        z=0,
        w=-0.707,
    )

    LEFTWARD_DIAG = Quaternion(
        x=0.5,
        y=-0.5,
        z=0,
        w=-0.707,
    )

    POSITION_1 = Quaternion(
        x=0.204954637077,
        y=0.978093304167,
        z=-0.0350698408324,
        w=0.00985856726981,
    )

    DOWNWARD_ROTATED = Quaternion(
        x=0.38706593901,
        y=0.921284010259,
        z=-0.000187850862603,
        w=-0.0376257360369,
    )

    SIDEWAYS_ORIENTED = Quaternion(
        x = 0.5,
        y = -0.5,
        z = 0.5,
        w = 0.5,
    )

    SLIGHT_RIGHT_20 = Quaternion(
        x=0.062,
        y=0.704,
        z=0.062,
        w=0.704,
    )

    SLIGHT_LEFT_20 = Quaternion(
        x=-0.062,
        y=0.704,
        z=-0.062,
        w=0.704,
    )

    SLIGHT_FRONT_20 = Quaternion(
        x=0.0,
        y=0.766,
        z=0.0,
        w=0.643,
    )

    SLIGHT_BACK_20 = Quaternion(
        x=0.0,
        y=0.643,
        z=0.0,
        w=0.766,
    )

    SLIGHT_RIGHT_60 = Quaternion(
        x=0.0,
        y=0.0,
        z=0.5,
        w=0.866,
    )

    SLIGHT_LEFT_60 = Quaternion(
        x=0.0,
        y=0.0,
        z=-0.5,
        w=0.866,
    )

    SLIGHT_BACK_60 = Quaternion(
        x=0.0,
        y=0.259,
        z=0.0,
        w=0.966,
    )

    SLIGHT_FRONT_60 = Quaternion(
        x=0.0,
        y=0.966,
        z=0.0,
        w=0.259,
    )

    SLIGHT_BACK_30 = Quaternion(
        x=0.0,
        y=0.5,
        z=0.0,
        w=0.866,
    )

    SLIGHT_FRONT_30 = Quaternion(
        x=0.0,
        y=0.866,
        z=0.0,
        w=0.5,
    )

    SLIGHT_LEFT_30 = Quaternion(
        x=0.0,
        y=0.0,
        z=-0.259,
        w=0.966,

    )

    SLIGHT_RIGHT_30 = Quaternion(
        x=0.0,
        y=0.0,
        z=0.259,
        w=0.966,
    )

    SLIGHT_BACK_45 = Quaternion(
        x=0.0,
        y=0.383,
        z=0.0,
        w=0.924,
    )

    SLIGHT_FRONT_45 = Quaternion(
        x=0.0,
        y=0.924,
        z=0.0,
        w=0.383,
    )

    SLIGHT_LEFT_45 = Quaternion(
        x=0.0,
        y=0.0,
        z=-0.383,
        w=0.924,

    )

    SLIGHT_RIGHT_45 = Quaternion(
        x=0.0,
        y=0.0,
        z=0.383,
        w=0.924,
    )

