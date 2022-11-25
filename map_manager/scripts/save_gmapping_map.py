#!/usr/bin/env python3

import argparse
import logging

import rospy
import rospy.impl.rosout
from nav_msgs.srv import GetMap, GetMapRequest, GetMapResponse
from sensor_msgs.msg import CompressedImage

from map_manager_msgs.msgs import MapInfo
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from sm_core.state import COMPLETED, FAILED

logger = logging.getLogger(__name__)


def task(map_name):
    save_map = rospy.ServiceProxy(name='/map_manager/add_map', service_class=AddMap)
    get_map = rospy.ServiceProxy(name='modular_gmapping/get_map', service_class=GetMap)

    save_map.wait_for_service(timeout=30)
    get_map.wait_for_service(timeout=30)

    # Get the OccupancyGrid
    try:
        get_map_response = get_map.call(GetMapRequest())  # type: GetMapResponse
    except rospy.ServiceException as e:
        logger.error('Failed to get map from gmapping: {}'.format(e))
        return FAILED

    # Save the map
    save_map_response = save_map.call(
        AddMapRequest(
            map_info=MapInfo(
                name=map_name,
                meta_data=get_map_response.map.info
            ),
            occupancy_grid=CompressedImage(
                format='raw',
                data=get_map_response.map.data
            )
        )
    )  # type: AddMapResponse

    if save_map_response.success is False:
        logger.error('Failed to save map: {}'.format(save_map_response.message))
        return FAILED

    return COMPLETED


if __name__ == '__main__':
    rospy.init_node('save_map', disable_signals=True)

    logger.setLevel(logging.INFO)
    handlers = logging.getLogger('rosout').handlers
    for handler in handlers:
        logger.addHandler(handler)

    parser = argparse.ArgumentParser()
    parser.add_argument('map_name', help='map_name')

    args = parser.parse_args(rospy.myargv()[1:])

    task(map_name=args.map_name)
