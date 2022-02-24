#!/usr/bin/env python3

import logging
import rospy.impl.rosout

from graph_planner.ros_wrapper import RosWrapper
logger = logging.getLogger(__name__)

name = 'graph_planner'

if __name__ == '__main__':

    rospy.init_node(name, disable_signals=True)

    handlers = logging.getLogger('rosout').handlers
    for handler in handlers:
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        for _logger_name, _logger in logging.Logger.manager.loggerDict.items():  # type: ignore
            if _logger_name not in ['rosgraph', 'rospy', 'rosout'] and '.' not in _logger_name:
                if isinstance(_logger, logging.PlaceHolder):
                    _logger = logging.getLogger(_logger_name)
                _logger.addHandler(handler)
                _logger.setLevel(logging.DEBUG)

    logger.info('Starting Graph Planner...')

    obj = RosWrapper()
    rospy.spin()
