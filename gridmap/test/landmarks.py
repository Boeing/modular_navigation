#!/usr/bin/env python
import logging

import rospy

from camera_interface.srv import GetImage, GetImageRequest, GetImageResponse

logger = logging.getLogger(__name__)


def run():
    get_image = rospy.ServiceProxy(name='camera/get_image', service_class=GetImage)

    while not rospy.is_shutdown():

        response = get_image.call(GetImageRequest(timeout=10))

        assert isinstance(response, GetImageResponse)

        print response.success


if __name__ == '__main__':
    rospy.init_node('landmarks')

    handlers = logging.getLogger('rosout').handlers
    for handler in handlers:
        logger.addHandler(handler)
        for _logger_name, _logger in logging.Logger.manager.loggerDict.items():  # type: ignore
            if _logger_name not in ['rosgraph', 'rospy', 'rosout'] and '.' not in _logger_name:
                if isinstance(_logger, logging.PlaceHolder):
                    _logger = logging.getLogger(_logger_name)
                _logger.addHandler(handler)
                _logger.setLevel(logging.DEBUG)

    run()
