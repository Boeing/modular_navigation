#!/usr/bin/env python3

# This script converts a DXF into a map components:
# MapInfo (JSON)
# Occupancy Grid (PNG)
# Pbstream (pbstream)
# Node Graph (JSON)
# Area Tree (JSON)
# Zones (JSON)

# Then it optionally uploads it to the map database either via the ROS interface or directly to MongoDB

import argparse
import logging
import mongoengine
import os

import rospy

from map_manager.test_utils import process_dxf
from map_manager.config import DATABASE_NAME

logger = logging.getLogger(__name__)

mongo_hostname = 'localhost'
mongo_port = 27017

node_name = 'dxf_to_database'


def dir_path(string):
    if string is None:
        return None
    else:
        if os.path.isdir(string):
            return string
        else:
            raise NotADirectoryError(string)


def run(dxf_file,
        cartographer_config,
        width_m,
        height_m,
        resolution,
        origin_x,
        origin_y,
        name,
        description,
        submap_locations,
        output_dir,
        plot,
        upload,
        node_name):

    process_dxf(
        dxf_file=dxf_file,
        cartographer_config=cartographer_config,
        width_m=width_m,
        height_m=height_m,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        name=name,
        description=description,
        submap_locations=submap_locations,
        output_dir=output_dir,
        plot=plot,
        upload=upload,
        node_name=node_name
    )

    logger.info('Done!')


if __name__ == '__main__':
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.INFO)
    logging.getLogger('map_manager.dxf.extract').addHandler(logging.StreamHandler())
    logging.getLogger('map_manager.dxf.extract').setLevel(logging.INFO)
    logging.getLogger('map_manager.test_utils').addHandler(logging.StreamHandler())
    logging.getLogger('map_manager.test_utils').setLevel(logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument('dxf', help='DXF file')
    parser.add_argument('cartographer_config', type=dir_path, help='Cartographer config directory')
    parser.add_argument('--width', type=float, default=64.0, help='Map width in metres')
    parser.add_argument('--height', type=float, default=64.0, help='Map height in metres')
    parser.add_argument('--resolution', type=float, default=0.02, help='Occupancy grid resolution in metres per cell')
    parser.add_argument('--origin_x', type=float, default=0.0, help='Map origin X (relative to world frame) in metres')
    parser.add_argument('--origin_y', type=float, default=0.0, help='Map origin Y (relative to world frame) in metres')
    parser.add_argument('--name', type=str, default='', help='Map name. Must be unique. Will use DXF name if not set')
    parser.add_argument('--description', type=str, default='', help='Map description')
    parser.add_argument('--submap_locations', default=None, nargs='+',
                        help='Submap location in the form x_center,y_center,x_size,y_size')
    parser.add_argument('--out', type=dir_path, default=None, help='Map output directory')
    parser.add_argument('--plot', action='store_true', help='Enable debug plotting')
    parser.add_argument('--upload', action='store_true', help='Upload to database. Use --node option to upload via ROS')
    parser.add_argument('--node', type=str, default=None,
                        help='ROS node name to upload via ROS interface. If not specified, map will be directly \
uploaded to Mongo.')

    args = parser.parse_args()

    if args.upload:
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
        else:
            logger.info('Connecting to {}:{}'.format(mongo_hostname, mongo_port))
            database = mongoengine.connect(db=DATABASE_NAME, host=mongo_hostname, port=mongo_port)
    else:
        if args.node:
            logger.warning('--node option used without --upload. --node will be ignored and map will not be uploaded.')

    print(args.submap_locations)

    if args.submap_locations is not None:
        submap_locations = list()
        for location_str in args.submap_locations:
            values = location_str.split(',')
            if len(values) != 4:
                logger.error('Submap location must be 4 numbers in the form x_center,y_center,x_size,y_size')
                exit(1)

            submap_locations.append([float(num_str) for num_str in values])
    else:
        submap_locations = None

    print(submap_locations)

    run(
        dxf_file=args.dxf,
        cartographer_config=args.cartographer_config,
        width_m=args.width,
        height_m=args.height,
        resolution=args.resolution,
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        name=args.name,
        description=args.description,
        submap_locations=submap_locations,
        output_dir=args.out,
        plot=args.plot,
        upload=args.upload,
        node_name=args.node
    )
