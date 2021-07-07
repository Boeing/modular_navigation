import logging
import os
import subprocess
import tempfile
from lxml import etree
from math import ceil

import rospy
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Point32 as Point32Msg
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import CompressedImage

from hd_map.msg import Map, MapInfo, Marker, Node, Path, Zone
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from math6d.geometry.quaternion import Quaternion

logger = logging.getLogger(__name__)


def get_zone_type(layer):
    if layer == 'exclusion_zone':
        return Zone.EXCLUSION_ZONE
    elif layer == 'drivable_zone':
        return Zone.DRIVABLE_ZONE
    elif layer == 'avoid_zone':
        return Zone.AVOID_ZONE
    else:
        return Zone.UNKNOWN


def generate_cartographer_map(
        cartographer_configuration_directory,
        world_sdf_path,
        resolution=0.02,
        map_origin=(-30, -10),
        map_size=(64, 64),
        map_free_space_size=10.0,
        submap_locations=((30, 22, 64, 32), (46, 22, 64, 32))
):
    assert os.path.isdir(cartographer_configuration_directory)
    assert os.path.isfile(world_sdf_path)

    temp_png_f = tempfile.NamedTemporaryFile()
    temp_pb_f = tempfile.NamedTemporaryFile()

    cmd = [
        'rosrun',
        'cartographer_ros',
        'sdf_to_pbstream',
        '--configuration_directory',
        cartographer_configuration_directory,
        '--pbstream',
        temp_pb_f.name,
        '--map_png',
        temp_png_f.name,
        '--world_sdf',
        world_sdf_path,
        '--map_origin',  # bottom left corner position
        '{},{}'.format(*map_origin),
        '--map_size',
        '{},{}'.format(*map_size),
        '--map_free_space_size',
        '{}'.format(map_free_space_size)
    ]

    for s in submap_locations:
        cmd += ['--submap_location', ','.join([str(_s) for _s in s])]

    str_cmd = ' '.join(cmd)

    subprocess.check_call(str_cmd, shell=True, stderr=subprocess.STDOUT)
    temp_pb_f.seek(0)
    pb_bytes = temp_pb_f.read()

    temp_png_f.seek(0)
    im_bytes = temp_png_f.read()

    xml_file = open(world_sdf_path, 'r')
    sdf_xml = xml_file.read()
    parser = etree.XMLParser(remove_blank_text=True)
    original_xml_element = etree.XML(sdf_xml, parser=parser)
    world = original_xml_element.find('world')

    #
    # Extract the zones
    #
    zones = []
    for model in world.findall('model'):
        links = model.findall('link')
        for link in links:
            layer = (link.attrib['layer'] if 'layer' in link.attrib else '').lower()
            zone_type = get_zone_type(layer)
            if zone_type != Zone.UNKNOWN:
                collision = link.find('visual')
                geometry = collision.find('geometry')
                polyline = geometry.find('polyline')
                points = [[float(t) for t in point.text.split(' ')] for point in polyline.findall('point')]
                zones.append(
                    Zone(
                        name=link.attrib['name'] if 'name' in link.attrib else layer,
                        zone_type=zone_type,
                        polygon=PolygonMsg(
                            points=[
                                Point32Msg(point[0], point[1], 0)
                                for point in points
                            ]
                        )
                    )
                )

    #
    # Extract the path nodes
    #
    nodes = []
    for model in world.findall('model'):
        for node in model.findall('node'):
            position = node.find('position')
            node_id = node.attrib['id']
            points = [float(t) for t in position.text.split(' ')]
            nodes.append(
                Node(
                    id=node_id,
                    x=points[0],
                    y=points[1]
                )
            )

    #
    # Extract the paths
    #
    paths = []
    for model in world.findall('model'):
        for path in model.findall('path'):
            node_ids = [ni.text for ni in path.findall('node')]
            paths.append(
                Path(
                    name='',
                    nodes=node_ids,
                )
            )

    #
    # Extract the markers
    #
    markers = []
    for model in world.findall('model'):
        links = model.findall('link')
        for link in links:
            layer = (link.attrib['layer'] if 'layer' in link.attrib else '').lower()
            if layer == 'marker':
                pose = link.find('pose')
                values = [float(f) for f in pose.text.split(' ')]
                if len(values) != 6:
                    raise Exception('Invalid Pose: {}'.format(values))
                qt = Quaternion.from_euler_extrinsic(*values[3:])
                markers.append(
                    Marker(
                        name=link.attrib['name'] if 'name' in link.attrib else layer,
                        marker_type=0,
                        pose=PoseMsg(
                            position=PointMsg(values[0], values[1], values[2]),
                            orientation=QuaternionMsg(x=qt.x, y=qt.y, z=qt.z, w=qt.w)
                        )
                    )
                )

    #
    # Save the map
    #
    add_map_srv = rospy.ServiceProxy(
        name='/map_manager/add_map',
        service_class=AddMap
    )

    add_map_srv.wait_for_service(timeout=10)
    add_response = add_map_srv.call(
        AddMapRequest(
            map=Map(
                info=MapInfo(
                    name='sim_map',
                    meta_data=MapMetaData(
                        resolution=resolution,
                        width=ceil(map_size[0] / resolution),
                        height=ceil(map_size[1] / resolution),
                        origin=PoseMsg(position=PointMsg(x=map_origin[0], y=map_origin[1]))
                    )
                ),
                markers=markers,
                zones=zones,
                paths=paths,
                nodes=nodes,
                default_zone=Zone.EXCLUSION_ZONE
            ),
            occupancy_grid=CompressedImage(
                format='png',
                data=im_bytes
            ),
            map_data=pb_bytes
        )
    )  # type: AddMapResponse
    if not add_response.success:
        raise Exception('Failed to save map: {}'.format(add_response.message))
