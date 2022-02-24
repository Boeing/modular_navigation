#!/usr/bin/env python3

# This script uploads all the map components in a folder to the map database
# Components:
# MapInfo (JSON)
# Occupancy Grid (PNG)
# Pbstream (pbstream)
# Node Graph (JSON)
# Area Tree (JSON)
# Zones (JSON)

import argparse
from datetime import datetime
import json
import logging
from PIL import Image
import mongoengine
import os
import io

import matplotlib.pyplot as plt
import networkx as nx
from networkx.drawing.nx_pydot import graphviz_layout

import rospy

from map_manager.documents import Map as MapDoc
from map_manager.documents import Point as PointDoc
from map_manager.documents import Pose as PoseDoc
from map_manager.documents import Quaternion as QuaternionDoc
from map_manager.documents import Zone as ZoneDoc
from map_manager.map_info import MapInfo as MapInfoCls

from graph_map.area import Zone
from graph_map.node_graph_manager import NodeGraphManager
from graph_map.area_manager import AreaManager
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from sensor_msgs.msg import CompressedImage

from map_manager.config import DATABASE_NAME

logger = logging.getLogger(__name__)

mongo_hostname = 'localhost'
mongo_port = 27017

node_name = 'upload_map'


def dir_path(string):
    if string is None:
        return None
    else:
        if os.path.isdir(string):
            return string
        else:
            raise NotADirectoryError(string)


def run(dir,
        plot,
        node_name):

    # Map Info
    if os.path.exists(os.path.join(dir, 'map_info.json')):
        with open(os.path.join(dir, 'map_info.json'), 'r') as fp:
            map_info = MapInfoCls.from_dict(json.load(fp))
            logger.info('Found "map_info.json" with map name: {}'.format(map_info.name))
    else:
        logger.error('Mandatory file "map_info.json" not found. Quitting.')
        exit(1)

    # Occupancy Grid
    if os.path.exists(os.path.join(dir, 'occupancy_grid.png')):
        with open(os.path.join(dir, 'occupancy_grid.png'), 'rb') as fp:
            og_bytes = fp.read()
    else:
        og_bytes = None
        logger.error('"occupancy_grid.png" not found')

    # Pbstream
    if os.path.exists(os.path.join(dir, 'cartographer_map.pbstream')):
        with open(os.path.join(dir, 'cartographer_map.pbstream'), 'rb') as fp:
            pb_bytes = fp.read()
    else:
        pb_bytes = None
        logger.warning('"cartographer_map.pbstream" not found')

    # Node graph
    if os.path.exists(os.path.join(dir, 'node_graph.json')):
        # We could just read as a string and upload as-is but this way we can verify we have a valid JSON
        gm = NodeGraphManager.read_graph(os.path.join(dir, 'node_graph.json'))
        logger.info('Found "node_graph.json" with {} nodes'.format(len(gm.get_nodes().keys())))
    else:
        gm = None
        logger.warning('"node_graph.json" not found')

    # Area Tree
    if os.path.exists(os.path.join(dir, 'area_tree.json')):
        # We could just read as a string and upload as-is but this way we can verify we have a valid JSON
        am = AreaManager.read_graph(os.path.join(dir, 'area_tree.json'))
        logger.info('Found "area_tree.json" with {} areas'.format(len(am.get_areas(level=1).keys())))
    else:
        am = None
        logger.warning('"area_tree.json" not found')

    # Zones
    if os.path.exists(os.path.join(dir, 'zones.json')):
        with open(os.path.join(dir, 'zones.json'), 'r') as fp:
            zone_dicts = json.load(fp)
            zones = [Zone.from_dict(zone_dict) for zone_dict in zone_dicts]
        logger.info('Found "zones.json" with {} zones'.format(len(zones)))
    else:
        zones = None
        logger.warning('"zones.json" not found')

    if plot:
        _, axs = plt.subplots(1, 2)

        label_dict = {node: node.display_name for node in gm.nodes}
        pos_dict = {node: (node.x, node.y) for node in gm.nodes}
        nx.draw(gm.graph, pos=pos_dict, labels=label_dict, with_labels=True, node_size=100, font_size=6, ax=axs[0])

        # Occupancy grid
        im = Image.open(io.BytesIO(og_bytes))
        im.show()
        del im

        # Plot nodes, zones and areas
        for zone in zones:
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

    #
    # Save the map via ROS interface
    #
    if node_name:
        logger.info('Uploading map via ROS to {}'.format(node_name))

        if gm is not None:
            gm_str = gm.to_json()
        else:
            gm_str = ''
        if am is not None:
            am_str = am.to_json()
        else:
            am_str = ''

        if zones is None:
            zones = list()

        add_map_srv = rospy.ServiceProxy(
            name=node_name + '/add_map',
            service_class=AddMap
        )

        add_map_srv.wait_for_service(timeout=10)
        add_response = add_map_srv.call(
            AddMapRequest(
                map_info=map_info.to_msg(),
                node_graph=gm_str,
                area_tree=am_str,
                zones=[zone.to_msg() for zone in zones],
                occupancy_grid=CompressedImage(
                    format='png',
                    data=og_bytes
                ),
                pbstream=pb_bytes
            )
        )  # type: AddMapResponse
        if not add_response.success:
            raise Exception('Failed to save map: {}'.format(add_response.message))

    #
    # Save to mongo database without ROS
    #
    else:
        logger.info('Uploading map directly to database')
        logger.info('Connecting to {}:{}'.format(mongo_hostname, mongo_port))
        mongoengine.connect(db=DATABASE_NAME, host=mongo_hostname, port=mongo_port)

        map_query = MapDoc.objects(name=map_info.name)
        if map_query.count():
            map_obj = map_query.first()
            logger.info('Map {} already exists in the database. Updating.'.format(map_info.name))
        else:
            map_obj = MapDoc()

        map_obj.name = map_info.name
        map_obj.description = map_info.description
        map_obj.modified = datetime.utcnow()
        map_obj.width = map_info.width  # width and height are stored as number of cells
        map_obj.height = map_info.height
        map_obj.resolution = map_info.resolution
        map_obj.origin = PoseDoc(
            position=PointDoc(
                x=map_info.origin_x,
                y=map_info.origin_y,
                z=0),
            quaternion=QuaternionDoc(
                x=0,
                y=0,
                z=0,
                w=1.0
            )
        )

        # Upload occupancy grid as PNG
        if og_bytes is not None:
            map_obj.image.replace(io.BytesIO(og_bytes))

            map_obj.thumbnail.delete()
            map_obj.thumbnail.new_file()
            im = Image.open(io.BytesIO(og_bytes))
            im.thumbnail(size=(400, 400))
            im.save(fp=map_obj.thumbnail, format='PPM')
            map_obj.thumbnail.close()

        # Upload pbstream
        if pb_bytes is not None:
            map_obj.pbstream.replace(io.BytesIO(pb_bytes))

        # Graph maps
        if gm is not None:
            map_obj.node_graph = gm.to_json()
        if am is not None:
            map_obj.area_tree = am.to_json()

        # Zones
        if zones is not None:
            map_obj.zones.clear()
            for zone in zones:
                map_obj.zones.append(ZoneDoc.from_msg(zone.to_msg()))

        map_obj.save()

    logger.info('Done!')


if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument('dir', type=dir_path, help='Directory containing map data')
    parser.add_argument('--plot', action='store_true', help='Enable debug plotting')
    parser.add_argument('--node', type=str, default=None, help='ROS node name to upload via ROS interface')

    args = parser.parse_args()

    if args.node:
        rospy.init_node(node_name, disable_signals=True)

        handlers = logging.getLogger('rosout').handlers
        for handler in handlers:
            logger.addHandler(handler)
            logger.setLevel(logging.INFO)
            for _logger_name, _logger in logging.Logger.manager.loggerDict.items():  # type: ignore
                if _logger_name not in ['rosgraph', 'rospy', 'rosout'] and '.' not in _logger_name:
                    if isinstance(_logger, logging.PlaceHolder):
                        _logger = logging.getLogger(_logger_name)
                    _logger.addHandler(handler)
                    _logger.setLevel(logging.INFO)

    run(
        dir=args.dir,
        plot=args.plot,
        node_name=args.node
    )
