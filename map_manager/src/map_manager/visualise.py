import logging
import math
from math3d.orientation import Orientation

from math6d.geometry import Vector3, Quaternion
import rospy
import typing
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import Vector3 as Vector3Msg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg

from map_manager import tess
from map_manager.documents import Map as MapDocument
from map_manager.documents import Zone as ZoneDocument
from hd_map.msg import Zone as ZoneMsg

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
        scale=Vector3Msg(
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
            scale=Vector3Msg(size / 6, size / 3, 0),
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
            scale=Vector3Msg(size / 6, size / 3, 0),
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
            scale=Vector3Msg(size / 6, size / 3, 0),
            color=ColorRGBA(0.0, 0.0, 1.0, 1.0)
        )
    ]


def build_zone_markers(zone):
    # type: (ZoneDocument) -> typing.Sequence[Marker]
    markers = list()  # type: typing.List[Marker]

    if zone.zone_type == ZoneMsg.EXCLUSION_ZONE:
        color = ColorRGBA(1.0, 0.0, 0.2, 0.2)
    elif zone.zone_type == ZoneMsg.DRIVABLE_ZONE:
        color = ColorRGBA(0.0, 1.0, 0.2, 0.1)
    elif zone.zone_type == ZoneMsg.AVOID_ZONE:
        color = ColorRGBA(1.0, 0.6, 0.0, 0.2)
    else:
        color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

    polygon_points = [
        (point.x, point.y, point.z)
        for point in zone.polygon
    ]
    triangles = tess.triangulate(polygon_points)
    triangle_marker = Marker(
        type=Marker.TRIANGLE_LIST,
        frame_locked=True,
        scale=Vector3Msg(1.0, 1.0, 1.0),
        color=color
    )
    for t in triangles:
        # Forward triangle
        triangle_marker.points.append(PointMsg(*polygon_points[t[0]]))
        triangle_marker.points.append(PointMsg(*polygon_points[t[1]]))
        triangle_marker.points.append(PointMsg(*polygon_points[t[2]]))

        # Reverse triangle
        triangle_marker.points.append(PointMsg(*polygon_points[t[0]]))
        triangle_marker.points.append(PointMsg(*polygon_points[t[2]]))
        triangle_marker.points.append(PointMsg(*polygon_points[t[1]]))

    markers.append(triangle_marker)
    return markers


def build_marker_array(map_obj):
    # type: (MapDocument) -> MarkerArray

    assert isinstance(map_obj, MapDocument)

    marker_array = MarkerArray()

    marker_array.markers.append(Marker(type=Marker.DELETEALL))

    for zone in map_obj.zones:
        marker_array.markers += build_zone_markers(zone=zone)

    nodes = {}
    for node in map_obj.nodes:
        nodes[node.id] = node
        marker_array.markers.append(
            Marker(
                type=Marker.SPHERE,
                pose=PoseMsg(
                    position=PointMsg(x=node.point.x, y=node.point.y, z=0),
                    orientation=QuaternionMsg(w=1)
                ),
                frame_locked=True,
                scale=Vector3Msg(0.06, 0.06, 0.06),
                color=ColorRGBA(0.0, 0.0, 1.0, 1.0)
            )
        )

    for path in map_obj.paths:
        for i in range(len(path.nodes) - 1):
            start_node = nodes[path.nodes[i]]
            end_node = nodes[path.nodes[i + 1]]
            marker_array.markers.append(
                build_cylinder_marker(
                    start_point=Vector3(start_node.point.x, start_node.point.y, 0),
                    end_point=Vector3(end_node.point.x, end_node.point.y, 0),
                    color=ColorRGBA(0.0, 0.0, 1.0, 1.0),
                    radius=0.02
                )
            )

    print marker_array

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
