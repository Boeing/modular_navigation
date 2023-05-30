import logging
import math
import typing

import networkx as nx
import rclpy
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from geometry_msgs.msg import Vector3 as Vector3Msg
from graph_map.area_manager import AreaManager
from graph_map.node import Node
from graph_map.node_graph_manager import NodeGraphManager
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from map_manager.documents import Color as ColorDoc
from map_manager.documents import Map as MapDoc
from map_manager.documents import Point as PointDoc
from map_manager.documents import Polygon as PolygonDoc
from map_manager.documents import Region as RegionDoc
from map_manager.tess import triangulate
from math6d.geometry import Quaternion, Vector3

logger = logging.getLogger(__name__)


def sort_clockwise(points):
    val = ((points[1][1] - points[0][1]) * (points[2][0] - points[1][0])) - \
          ((points[1][0] - points[0][0]) * (points[2][1] - points[1][1]))

    if (val < 0):
        # Counterclockwise orientation
        return [points[0], points[2], points[1]]
    else:
        # Collinear or clockwise orientation
        return points


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
            x=float(radius),
            y=float(radius),
            z=float(vec.length())
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
                x=pose.position.x + mat[:, 0][0] * size,
                y=pose.position.y + mat[:, 0][1] * size,
                z=pose.position.z + mat[:, 0][2] * size
            )],
            frame_locked=True,
            scale=Vector3Msg(x=size / 6, y=size / 3, z=0),
            color=ColorRGBA(1.0, 0, 0, 1.0)
        ),
        Marker(
            type=Marker.ARROW,
            points=[pose.position, PointMsg(
                x=pose.position.x + mat[:, 1][0] * size,
                y=pose.position.y + mat[:, 1][1] * size,
                z=pose.position.z + mat[:, 1][2] * size
            )],
            frame_locked=True,
            scale=Vector3Msg(x=size / 6, y=size / 3, z=0),
            color=ColorRGBA(0.0, 1.0, 0, 1.0)
        ),
        Marker(
            type=Marker.ARROW,
            points=[pose.position, PointMsg(
                x=pose.position.x + mat[:, 2][0] * size,
                y=pose.position.y + mat[:, 2][1] * size,
                z=pose.position.z + mat[:, 2][2] * size
            )],
            frame_locked=True,
            scale=Vector3Msg(x=size / 6, y=size / 3, z=0),
            color=ColorRGBA(0.0, 0.0, 1.0, 1.0)
        )
    ]


def build_node_markers(node: Node, color, radius, height, lifetime):
    markers = list()  # type: typing.List[Marker]

    up_vec = Vector3(0, 0, 1)
    qt = Quaternion.from_axis_angle(axis=up_vec, angle=0)

    markers.append(
        Marker(
            type=Marker.CYLINDER,
            pose=PoseMsg(
                position=PointMsg(x=node.x, y=node.y, z=0.0),
                orientation=QuaternionMsg(
                    x=qt.x, y=qt.y, z=qt.z, w=qt.w
                )
            ),
            frame_locked=True,
            scale=Vector3Msg(
                x=float(radius),
                y=float(radius),
                z=float(height)
            ),
            color=color,
            lifetime=lifetime
        )
    )

    return markers


# Lifetime is of type builtin_interfaces.Duration
def build_edge_marker(start: Node, end: Node, color: ColorRGBA, diameter: float, lifetime):
    return Marker(
        type=Marker.ARROW,
        points=[
            PointMsg(
                x=start.x,
                y=start.y,
                z=0.
            ),
            PointMsg(
                x=end.x,
                y=end.y,
                z=0.
            ),
        ],
        pose=PoseMsg(
            orientation=QuaternionMsg(x=0., y=0., z=0., w=1.)
        ),
        scale=Vector3Msg(
            x=float(diameter),
            y=float(diameter * 6),
            z=0.5
        ),
        frame_locked=True,
        color=color,
        lifetime=lifetime
    )


def build_region_markers(region):
    # type: (RegionDoc) -> typing.Sequence[Marker]
    markers = list()  # type: typing.List[Marker]

    polygon_points = [
        (point.x, point.y, point.z)
        for point in region.polygon.points
    ]

    triangles = triangulate(polygon_points)

    pose = PoseMsg(
        orientation=QuaternionMsg(x=0., y=0., z=0., w=1.)
    )
    triangle_marker = Marker(
        type=Marker.TRIANGLE_LIST,
        frame_locked=True,
        pose=pose,
        scale=Vector3Msg(x=1.0, y=1.0, z=1.0),
        color=region.color.get_msg()
    )

    for t in triangles:
        points = [polygon_points[t[0]], polygon_points[t[1]], polygon_points[t[2]]]
        points = sort_clockwise(points)

        # triangle_marker.points.append(PointMsg(*points[2]))
        # triangle_marker.points.append(PointMsg(*points[1]))
        # triangle_marker.points.append(PointMsg(*points[0]))

        triangle_marker.points.append(PointMsg(x=points[2][0], y=points[2][1], z=points[2][2]))
        triangle_marker.points.append(PointMsg(x=points[1][0], y=points[1][1], z=points[1][2]))
        triangle_marker.points.append(PointMsg(x=points[0][0], y=points[0][1], z=points[0][2]))

    markers.append(triangle_marker)
    return markers


def build_zones_marker_array(node, map_obj: MapDoc) -> MarkerArray:

    assert isinstance(map_obj, MapDoc)

    marker_array = MarkerArray()

    pose = PoseMsg(orientation=QuaternionMsg(x=0., y=0., z=0., w=1.))
    marker_array.markers.append(
        Marker(
            type=Marker.DELETEALL,
            pose=pose,
            scale=Vector3Msg(x=1.0, y=1.0, z=1.0),
            color=ColorRGBA(r=0., g=0., b=0., a=1.0)
        )
    )

    for zone in map_obj.zones:
        for region in zone.regions:
            marker_array.markers += build_region_markers(region=region)
            # break

    now = node.get_clock().now().to_msg()
    for i, marker in enumerate(marker_array.markers):
        assert isinstance(marker, Marker)
        marker.id = i
        marker.header.frame_id = 'map'
        marker.header.stamp = now

    return marker_array


def build_areas_marker_array(node, map_obj):
    # type: (Node, MapDoc) -> MarkerArray

    # return MarkerArray()

    assert isinstance(map_obj, MapDoc)

    marker_array = MarkerArray()
    pose = PoseMsg(orientation=QuaternionMsg(x=0., y=0., z=0., w=1.))
    marker_array.markers.append(
        Marker(
            type=Marker.DELETEALL,
            pose=pose,
            scale=Vector3Msg(x=1., y=1., z=1.),
            color=ColorRGBA(r=0., g=0., b=0., a=1.0)
        )
    )

    if map_obj.area_tree is not None and map_obj.area_tree != '':
        am = AreaManager.from_jsons(map_obj.area_tree)

        # We are constructing the AreaManager from JSON, so the regions will need to be converted into RegionDocuments
        for area in am.get_areas(level=1):
            for region in area.regions:
                region_doc = RegionDoc(
                    polygon=PolygonDoc(points=[PointDoc(x=p[0], y=p[1], z=0.0) for p in region.points]),
                    color=ColorDoc(r=region.color.r, g=region.color.g, b=region.color.b, a=region.color.a)
                )
                marker_array.markers += build_region_markers(region=region_doc)

    now = node.get_clock().now().to_msg()
    for i, marker in enumerate(marker_array.markers):
        assert isinstance(marker, Marker)
        marker.id = i
        marker.header.frame_id = 'map'
        marker.header.stamp = now

    return marker_array


def build_graph_marker_array(ros_node,
                             map_obj: typing.Union[MapDoc, NodeGraphManager, nx.Graph],
                             node_params: typing.Optional[typing.Dict] = None,
                             edge_params: typing.Optional[typing.Dict] = None) -> MarkerArray:
    node_params_ = {'color': ColorRGBA(r=0., g=0., b=1.0, a=0.7), 'radius': 0.3,
                    'height': 0.1, 'lifetime': rclpy.duration.Duration(seconds=6).to_msg()}
    edge_params_ = {'color': ColorRGBA(r=0., g=1.0, b=0., a=0.3), 'diameter': 0.05,
                    'lifetime': rclpy.duration.Duration(seconds=6).to_msg()}

    if node_params is not None:
        node_params_.update(node_params)
    if edge_params is not None:
        edge_params_.update(edge_params)

    marker_array = MarkerArray()
    pose = PoseMsg(orientation=QuaternionMsg(x=0., y=0., z=0., w=1.))
    marker_array.markers.append(
        Marker(
            type=Marker.DELETEALL,
            pose=pose,
            scale=Vector3Msg(x=1., y=1., z=1.),
            color=ColorRGBA(r=0., g=0., b=0., a=1.0),
            lifetime=rclpy.duration.Duration(seconds=2).to_msg()
        )
    )

    if isinstance(map_obj, MapDoc):
        if map_obj.node_graph is not None and map_obj.area_tree != '':
            gm = NodeGraphManager.from_jsons(map_obj.node_graph)
            nodes = gm.get_nodes().keys()
            edges = gm.graph.edges
        else:
            nodes = []
            edges = []
    elif isinstance(map_obj, NodeGraphManager):
        gm = map_obj
        nodes = gm.get_nodes().keys()
        edges = gm.graph.edges
    elif isinstance(map_obj, nx.Graph):
        nodes = map_obj.nodes().keys()
        edges = map_obj.edges

    for node in nodes:
        marker_array.markers += build_node_markers(node, **node_params_)

    for start, end in edges:
        marker_array.markers.append(build_edge_marker(start, end, **edge_params_))

    now = ros_node.get_clock().now().to_msg()
    for i, marker in enumerate(marker_array.markers):
        assert isinstance(marker, Marker)
        marker.id = i
        marker.header.frame_id = 'map'
        marker.header.stamp = now

    return marker_array
