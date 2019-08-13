import logging
import math
from math3d.orientation import Orientation

from math6d.geometry import Vector3, Quaternion
import rospy
import typing
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg

from map_manager.documents import Map as MapDocument
from map_manager.documents import Zone as ZoneDocument


logger = logging.getLogger(__name__)


def build_cylinder_marker(start_point, end_point, color, radius):
    # type: (Vector3, Vector3, ColorRGBA, float) -> Marker
    vec = Vector3(end_point - start_point)
    center = (start_point + end_point) * (1 / 2.0)

    axis_vec = end_point - center
    axis_vec.normalize()
    up_vec = Vector3(0, 0, 1)
    right_axis_vector = axis_vec.cross(up_vec)
    right_axis_vector.normalize()

    theta = axis_vec * up_vec
    angle_rotation = -1.0 * math.acos(theta)

    qt = Quaternion.from_axis_angle(axis=right_axis_vector, angle=angle_rotation)

    return Marker(
        type=Marker.CYLINDER,
        pose=PoseMsg(
            position=PointMsg(*center.data()),
            orientation=QuaternionMsg(
                qt.x, qt.y, qt.z, qt.w
            )
        ),
        frame_locked=True,
        scale=Vector3(
            radius,
            radius,
            vec.length()
        ),
        color=color
    )


def build_pose_markers(pose, size=0.05):
    # type: (PoseMsg, float) -> typing.Sequence[Marker]
    quat_m3d = Quaternion(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z
    )
    mat = quat_m3d.matrix()
    return [
        Marker(
            type=Marker.ARROW,
            points=[pose.position, PointMsg(
                pose.position.x + mat[:, 0][0] * size,
                pose.position.y + mat[:, 0][1] * size,
                pose.position.z + mat[:, 0][2] * size
            )],
            frame_locked=True,
            scale=Vector3(size / 6, size / 3, 0),
            color=ColorRGBA(1.0, 0, 0, 1.0)
        ),
        Marker(
            type=Marker.ARROW,
            points=[pose.position, PointMsg(
                pose.position.x + mat[:, 1][0] * size,
                pose.position.y + mat[:, 1][1] * size,
                pose.position.z + mat[:, 1][2] * size
            )],
            frame_locked=True,
            scale=Vector3(size / 6, size / 3, 0),
            color=ColorRGBA(0.0, 1.0, 0, 1.0)
        ),
        Marker(
            type=Marker.ARROW,
            points=[pose.position, PointMsg(
                pose.position.x + mat[:, 2][0] * size,
                pose.position.y + mat[:, 2][1] * size,
                pose.position.z + mat[:, 2][2] * size
            )],
            frame_locked=True,
            scale=Vector3(size / 6, size / 3, 0),
            color=ColorRGBA(0.0, 0.0, 1.0, 1.0)
        )
    ]


def build_zone_markers(zone, color, radius):
    # type: (ZoneDocument, ColorRGBA, float) -> typing.Sequence[Marker]
    markers = list()  # type: typing.List[Marker]

    for i in range(len(zone.polygon) - 1):
        markers.append(
            build_cylinder_marker(
                start_point=Vector3(
                    zone.polygon[i].x,
                    zone.polygon[i].y,
                    zone.polygon[i].z
                ),
                end_point=Vector3(
                    zone.polygon[i + 1].x,
                    zone.polygon[i + 1].y,
                    zone.polygon[i + 1].z
                ),
                color=color,
                radius=radius
            )
        )
    if len(zone.polygon) > 0:
        markers.append(
            build_cylinder_marker(
                start_point=Vector3(
                    zone.polygon[-1].x,
                    zone.polygon[-1].y,
                    zone.polygon[-1].z
                ),
                end_point=Vector3(
                    zone.polygon[0].x,
                    zone.polygon[0].y,
                    zone.polygon[0].z
                ),
                color=color,
                radius=radius
            )
        )
    return markers


def build_marker_array(map_obj):
    # type: (MapDocument) -> MarkerArray

    assert isinstance(map_obj, MapDocument)

    marker_array = MarkerArray()

    for zone in map_obj.zones:
        marker_array.markers += build_zone_markers(
            zone=zone,
            color=ColorRGBA(0.0, 1.0, 0.0, 1.0),
            radius=0.04
        )

    for marker in map_obj.markers:
        marker_array.markers += build_pose_markers(
            pose=marker.pose.get_msg(),
            size=0.2
        )

    now = rospy.Time(0)
    for i, marker in enumerate(marker_array.markers):
        assert isinstance(marker, Marker)
        marker.id = i
        marker.header.frame_id = 'map'
        marker.header.stamp = now

    return marker_array
