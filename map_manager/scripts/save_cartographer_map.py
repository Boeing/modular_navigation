#!/usr/bin/env python

import argparse
import logging

import rospy
import rospy.impl.rosout

from cartographer_ros_msgs.srv import WriteState, WriteStateRequest, WriteStateResponse
from hd_map.msg import Map, MapInfo
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse

logger = logging.getLogger(__name__)


def task(map_name, resolution):
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
    save_map_response = save_map.call(
        AddMapRequest(
            map=Map(
                info=MapInfo(
                    name=map_name,
                    meta_data=get_map_response.map_info
                )
            ),
            occupancy_grid=get_map_response.occupancy_grid,
            map_data=get_map_response.pbstream_data
        )
    )  # type: AddMapResponse

    if save_map_response.success is False:
        logger.error('Failed to save map: {}'.format(save_map_response.message))
        exit(1)


if __name__ == '__main__':
    rospy.init_node('save_map', disable_signals=True)

    logger.setLevel(logging.INFO)
    handlers = logging.getLogger('rosout').handlers
    for handler in handlers:
        logger.addHandler(handler)

    parser = argparse.ArgumentParser()
    parser.add_argument('map_name', help='map_name', type=str)
    parser.add_argument('resolution', help='resolution', type=float)

    args = parser.parse_args(rospy.myargv()[1:])

    task(map_name=args.map_name, resolution=args.resolution)
