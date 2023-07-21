#!/usr/bin/env python3

import logging

import flask
import threading
import rclpy

from map_manager.ros_wrapper import RosWrapper
from map_manager.http_utils.routes import map_api
from map_manager.config import RESOURCE_PORT


def main(args=None):
    rclpy.init()

    map_manager_node = RosWrapper()

    logger = logging.getLogger(__name__)

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

    logger = map_manager_node.get_logger()

    logger.info('Starting Map Manager...')

    app = flask.Flask(__name__, template_folder='/map_manager/map_manager/http_utils')
    server_name = '0.0.0.0:' + str(RESOURCE_PORT)

    logger.info('Map Manager at ' + server_name)
    app.config['SERVER_NAME'] = server_name

    app.logger.setLevel(logging.INFO)

    for handler in handlers:
        app.logger.addHandler(handler)

    app.register_blueprint(map_api)

    server_thread = threading.Thread(target=app.run, daemon=True)
    server_thread.start()

    logger.info('Spinning map manager')

    rclpy.spin(map_manager_node)

    # server_thread.kill()
    map_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
