#!/usr/bin/env python3

import argparse
from datetime import datetime
import json
import logging
import os

import rospy
import rospy.impl.rosout

from cartographer_ros_msgs.srv import WriteState, WriteStateRequest, WriteStateResponse
from map_manager_msgs.msgs import MapInfo
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from map_manager.map_info import MapInfo as MapInfoCls

logger = logging.getLogger(__name__)


def dir_path(string):
    if string is None:
        return None
    else:
        if os.path.isdir(string):
            return string
        else:
            raise NotADirectoryError(string)


def task(map_name, resolution, out, upload):
    assert isinstance(map_name, str)
    assert isinstance(resolution, float)

    save_map = rospy.ServiceProxy(name='/map_manager/add_map', service_class=AddMap)
    get_map = rospy.ServiceProxy(name='/mapper/write_state', service_class=WriteState)

    save_map.wait_for_service(timeout=2)

    # Get the Map
    try:
        get_map_response = get_map.call(
            WriteStateRequest(
                resolution=resolution
            )
        )  # type: WriteStateResponse
    except rospy.ServiceException as e:
        logger.error('Failed to get map from cartographer: {}'.format(e))
        exit(1)

    # Save the map
    map_info_msg = MapInfo(name=map_name,
                           created=rospy.Time.from_sec(datetime.utcnow().timestamp()),
                           modified=rospy.Time.from_sec(datetime.utcnow().timestamp()),
                           meta_data=get_map_response.map_info)

    if upload:
        save_map_response = save_map.call(
            AddMapRequest(
                map_info=map_info_msg,
                occupancy_grid=get_map_response.occupancy_grid,
                pbstream=get_map_response.pbstream_data
            )
        )  # type: AddMapResponse

        if save_map_response.success is False:
            logger.error('Failed to save map: {}'.format(save_map_response.message))
        else:
            logger.info('Uploaded map to database')

    if out is not None:
        # Map Info
        map_info = MapInfoCls.from_msg(map_info_msg)
        with open(os.path.join(out, 'map_info.json'), 'w') as fp:
            json.dump(map_info.to_simple_dict(), fp, indent=4)

        # Occupancy Grid
        with open(os.path.join(out, 'occupancy_grid.png'), 'wb') as fp:
            fp.write(get_map_response.occupancy_grid.data)

        # Pbstream
        with open(os.path.join(out, 'cartographer_map.pbstream'), 'wb') as fp:
            fp.write(get_map_response.pbstream_data)

        logger.info('Save map files to: {}'.format(os.path.abspath(out)))


if __name__ == '__main__':
    rospy.init_node('save_map', disable_signals=True)

    logger.setLevel(logging.INFO)
    handlers = logging.getLogger('rosout').handlers
    for handler in handlers:
        logger.addHandler(handler)

    parser = argparse.ArgumentParser()
    parser.add_argument('map_name', help='map_name', type=str)
    parser.add_argument('resolution', help='resolution', type=float)
    parser.add_argument('--out', help='Output directory', default=None, type=dir_path)
    parser.add_argument('--upload', help='Output directory', action='store_true')

    args = parser.parse_args(rospy.myargv()[1:])

    task(map_name=args.map_name, resolution=args.resolution, out=args.out, upload=args.upload)
