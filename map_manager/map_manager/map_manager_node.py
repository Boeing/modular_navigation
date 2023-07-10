#!/usr/bin/env python3

import logging

import flask
import signal
import sys
from functools import partial
from multiprocessing import Process
# import mongoengine
# import pymongo
# import rospy.impl.rosout
import rclpy

from map_manager.ros_wrapper import RosWrapper
from map_manager.http_utils.routes import map_api
from threading import Thread
from map_manager.config import RESOURCE_PORT  # , DATABASE_NAME


def signal_handler(server: Process, sig, frame):
    server.terminate()
    server.join(timeout=5)
    sys.exit(0)


if __name__ == '__main__':
    # rospy.init_node(name, disable_signals=True)
    rclpy.init()

    map_manager_node = RosWrapper()
    # node = rclpy.create_node(name)

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

    # mongo_hostname = rospy.get_param('~mongo_hostname', 'localhost')
    # mongo_port = rospy.get_param('~mongo_port', 27017)

    # Parameters are now handled at each node, retrieve them from map_manager_node
    mongo_hostname = map_manager_node.get_parameter('~mongo_hostname').value
    mongo_port = map_manager_node.get_parameter('~mongo_port').value

    # rewrite logger from above
    logger = map_manager_node.get_logger()

    logger.info('Starting Map Manager...')

    #
    # Connect to the db
    #
    logger.info('Connecting to {}:{}'.format(mongo_hostname, mongo_port))
    # try:
    #    database = mongoengine.connect(
    #        db=DATABASE_NAME,
    #        host=mongo_hostname,
    #        port=mongo_port,
    #        serverSelectionTimeoutMS=30)
    # except mongoengine.ConnectionFailure as e:
    #    logger.error("Failed to connect to Mongodb", exc_info=e)
    #    raise e

    #
    # Force check to make sure Mongo is alive
    #
    # try:
    #    server = database.server_info()
    # except pymongo.errors.ServerSelectionTimeoutError as e:
    #    logger.error("Mongodb is offline", exc_info=e)
    #    raise e

    app = flask.Flask(__name__, template_folder='/map_manager/map_manager/http_utils')
    server_name = '0.0.0.0:' + str(RESOURCE_PORT)

    logger.info('Map Manager at ' + server_name)
    app.config['SERVER_NAME'] = server_name

    app.logger.setLevel(logging.INFO)
    for handler in handlers:
        app.logger.addHandler(handler)

    app.register_blueprint(map_api)

    server = Process(target=app.run)
    signal.signal(signal.SIGINT, partial(signal_handler, server))
    server.start()

    rclpy.spin(map_manager_node)  # This one doesnt get called

    map_manager_node.destroy_node()
    rclpy.shutdown()
