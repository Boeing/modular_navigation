import os
import subprocess
import tempfile
import json
from datetime import datetime
from math import ceil
from typing import Optional, List
from PIL import Image
from shutil import copy2

import matplotlib.pyplot as plt
import networkx as nx
from networkx.drawing.nx_pydot import graphviz_layout

from map_manager.dxf.dxf_loader import DxfLoader
from map_manager.documents import Map as MapDoc
from map_manager.documents import Point as PointDoc
from map_manager.documents import Pose as PoseDoc
from map_manager.documents import Quaternion as QuaternionDoc
from map_manager.documents import Zone as ZoneDoc
from map_manager.map_info import MapInfo as MapInfoCls

import rclpy
from graph_map.area import Zone
from map_manager.msg import MapInfo as MapInfoMsg
from map_manager.srv import AddMap
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import MapMetaData as MapMetaDataMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg


def sdf_to_og_pbstream(
        cartographer_configuration_directory,
        world_sdf_path,
        png_file,
        pb_file,
        map_origin=(0, 0),
        map_size=(32, 32),
        map_resolution=0.02,
        submap_locations=None,
        zones: Optional[List[Zone]] = None
):
    assert os.path.isdir(cartographer_configuration_directory)
    assert os.path.isfile(world_sdf_path)

    cmd = [
        'ros2',
        'run',
        'cartographer_ros',
        'sdf_to_pbstream',
        '--configuration_directory',
        cartographer_configuration_directory,
        '--pbstream',
        pb_file,
        '--map_png',
        png_file,
        '--world_sdf',
        world_sdf_path,
        '--map_origin',  # bottom left corner position
        '{},{}'.format(*map_origin),
        '--map_size',
        '{},{}'.format(*map_size),
        '--map_resolution',
        '{}'.format(map_resolution),
    ]

    if submap_locations is None:
        submap_locations = [(map_size[0]/2.0, map_size[1]/2.0, map_size[0], map_size[1])]

    for s in submap_locations:
        cmd += ['--submap_location', ','.join([str(_s) for _s in s])]

    if zones is None:
        zones = list()

    # Get the drivable zones and turn them into a list of free spaces
    # Each free space is a list of floats in the form [x1, y1, x2, y2, x3, y3 ...]
    free_spaces = list()
    for zone in zones:
        if zone.drivable:
            for region in zone.regions:
                free_space = list()
                for point in region.points:
                    free_space.append(point[0])
                    free_space.append(point[1])

                free_spaces.append(free_space)

    for f in free_spaces:
        cmd += ['--free_space', ','.join([str(_f) for _f in f])]

    str_cmd = ' '.join(cmd)

    subprocess.check_call(str_cmd, shell=True, stderr=subprocess.STDOUT)


def process_dxf(
        node,  # type: rclpy.Node
        dxf_file,
        cartographer_config,
        width_m,
        height_m,
        resolution,
        origin_x,
        origin_y,
        name='',
        description='',
        submap_locations=None,
        output_dir=None,
        plot=False,
        upload=True,
        node_name=None):

    logger = node.get_logger()

    if name is None or name == '':
        name = os.path.splitext(os.path.basename(dxf_file))[0]

    loader = DxfLoader(dxf_file)
    gm = loader.parse_dxf()
    am = gm.area_manager

    with tempfile.NamedTemporaryFile() as temp_sdf_f, \
            tempfile.NamedTemporaryFile() as temp_png_f, \
            tempfile.NamedTemporaryFile() as temp_pb_f:

        #
        # Generate SDF
        #
        sdf_path = temp_sdf_f.name
        loader.write_sdf(output_file=sdf_path)
        logger.info('Wrote SDF to {}'.format(os.path.abspath(sdf_path)))

        #
        # Convert SDF to OG and pbstream
        #
        sdf_to_og_pbstream(
            cartographer_configuration_directory=cartographer_config,
            world_sdf_path=sdf_path,
            png_file=temp_png_f.name,
            pb_file=temp_pb_f.name,
            map_origin=(origin_x, origin_y),
            map_size=(width_m, height_m),
            map_resolution=resolution,
            submap_locations=submap_locations,
            zones=loader.zones.values(),
        )

        temp_png_f.seek(0)
        im = Image.open(temp_png_f)
        im.load()

        #
        # Plotting
        #
        if plot:
            _, axs = plt.subplots(1, 2)

            label_dict = {node: node.display_name for node in gm.nodes}
            pos_dict = {node: (node.x, node.y) for node in gm.nodes}
            nx.draw(gm.graph, pos=pos_dict, labels=label_dict, with_labels=True, node_size=100, font_size=6, ax=axs[0])

            # Occupancy grid
            im.show()

            # Plot nodes, zones and areas
            for zone in loader.zones.values():
                zone.plot(ax=axs[0])
            for area in am.areas.keys():
                area.plot(ax=axs[0])

            axs[0].axis('equal')
            axs[0].axis('on')
            axs[0].tick_params(left=True, bottom=True, labelbottom=True, labelleft=True)

            # Plot area tree
            label_dict = {area: area.display_name for area in am.get_areas(level=None)}
            pos = graphviz_layout(am.tree, prog="dot")
            nx.draw(am.tree, pos, labels=label_dict, with_labels=True, font_size=6, ax=axs[1])

            plt.show()

        t = node.get_clock().now()
        map_info_msg = MapInfoMsg(
            name=name,
            description=description,
            created=t.to_msg(),
            modified=t.to_msg(),
            meta_data=MapMetaDataMsg(
                resolution=resolution,
                width=ceil(width_m / resolution),
                height=ceil(height_m / resolution),
                origin=PoseMsg(
                    position=PointMsg(x=float(origin_x), y=float(origin_y), z=0.0),
                    orientation=QuaternionMsg(x=0.0, y=0.0, z=0.0, w=1.0)
                )
            )
        )

        #
        # Save the map via ROS interface
        #
        if node_name and upload:
            logger.info('Uploading map via ROS to {}'.format(node_name))

            add_map_srv = node.create_client(
                AddMap,
                node_name + '/add_map'
            )

            temp_png_f.seek(0)
            temp_pb_f.seek(0)

            add_map_srv.wait_for_service(timeout_sec=5.0)

            add_req = AddMap.Request(
                map_info=map_info_msg,
                node_graph=gm.to_json(),
                area_tree=am.to_json(),
                zones=[zone.to_msg() for zone in loader.zones.values()],
                occupancy_grid=CompressedImage(
                    format='png',
                    data=temp_png_f.read()
                ),
                pbstream=temp_pb_f.read()
            )

            add_map_future = add_map_srv.call_async(add_req)
            rclpy.spin_until_future_complete(node, add_map_future)

            add_map_res = add_map_future.result()  # type: AddMap.Response

            if not add_map_res.success:
                raise Exception('Failed to save map: {}'.format(add_map_res.message))

        #
        # Save to mongo database without ROS
        #
        elif upload:
            logger.info('Uploading map directly to database')

            map_query = MapDoc.objects(name=map_info_msg.name)
            if map_query.count():
                map_obj = map_query.first()
                logger.info('Map {} already exists in the database. Updating.'.format(map_info_msg.name))
            else:
                map_obj = MapDoc()

            map_obj.name = map_info_msg.name
            map_obj.description = description
            map_obj.modified = datetime.utcnow()
            map_obj.width = ceil(width_m / resolution)  # width and height are stored as number of cells
            map_obj.height = ceil(height_m / resolution)
            map_obj.resolution = resolution
            map_obj.origin = PoseDoc(
                position=PointDoc(
                    x=origin_x,
                    y=origin_y,
                    z=0),
                quaternion=QuaternionDoc(
                    x=0,
                    y=0,
                    z=0,
                    w=1.0
                )
            )

            # Upload occupancy grid as PNG
            map_obj.image.delete()
            map_obj.image.new_file()
            im.save(fp=map_obj.image, format='PPM')
            map_obj.image.close()

            map_obj.thumbnail.delete()
            map_obj.thumbnail.new_file()
            im_copy = im.copy()
            im_copy.thumbnail(size=(400, 400))
            im_copy.save(fp=map_obj.thumbnail, format='PPM')
            map_obj.thumbnail.close()

            # Upload pbstream
            map_obj.pbstream.replace(temp_pb_f.read())

            # Graph maps
            map_obj.node_graph = gm.to_json()
            map_obj.area_tree = am.to_json()

            # Zones
            map_obj.zones.clear()
            for zone in loader.zones.values():
                map_obj.zones.append(ZoneDoc.from_msg(zone.to_msg()))

            map_obj.save()

        #
        # Save artifacts
        #
        if output_dir is not None:
            # Create directory with dxf_name
            dxf_name = os.path.splitext(os.path.basename(dxf_file))[0]
            artifacts_dir = os.path.join(output_dir, dxf_name)
            if not os.path.exists(artifacts_dir):
                os.makedirs(artifacts_dir)

            # Map Info
            map_info = MapInfoCls.from_msg(map_info_msg)
            with open(os.path.join(artifacts_dir, 'map_info.json'), 'w') as fp:
                json.dump(map_info.to_simple_dict(), fp, indent=4)

            # SDF
            copy2(sdf_path, os.path.join(artifacts_dir, 'sim_map.world'))

            # Occupancy Grid
            im.save(os.path.join(artifacts_dir, 'occupancy_grid.png'))

            # Pbstream
            copy2(temp_pb_f.name, os.path.join(artifacts_dir, 'cartographer_map.pbstream'))

            # Node graph
            gm.write_graph(os.path.join(artifacts_dir, 'node_graph.json'))

            # Area Tree
            am.write_graph(os.path.join(artifacts_dir, 'area_tree.json'))

            # Zones
            zone_dicts = [zone.to_simple_dict() for zone in loader.zones.values()]
            with open(os.path.join(artifacts_dir, 'zones.json'), 'w') as fp:
                json.dump(zone_dicts, fp, indent=4)

            # DXF (to keep everything together)
            copy2(dxf_file, os.path.join(artifacts_dir, os.path.basename(dxf_file)))
