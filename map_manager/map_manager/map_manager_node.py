#!/usr/bin/env python3

import logging

import flask
import mongoengine
import pymongo
#import rospy.impl.rosout
import rclpy

from map_manager.config import DATABASE_NAME, RESOURCE_PORT
from map_manager.ros_wrapper import RosWrapper
from map_manager.http_utils.routes import map_api



logger = logging.getLogger(__name__)

name = 'map_manager'

if __name__ == '__main__':
    #rospy.init_node(name, disable_signals=True)
    rclpy.init()
    node = rclpy.create_node(name)

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

    #mongo_hostname = rospy.get_param('~mongo_hostname', 'localhost')
    #mongo_port = rospy.get_param('~mongo_port', 27017)

    # Parameters are now handled at each node, no more param server
    mongo_hostname = node.declare_parameter('~mongo_hostname', 'localhost').value
    mongo_port = node.declare_parameter('~mongo_port', 27017).value

    # rewrite logger from above
    logger = node.get_logger()

    logger.info('Starting Map Manager...')

    #
    # Connect to the db
    #
    logger.info('Connecting to {}:{}'.format(mongo_hostname, mongo_port))
    try:
        database = mongoengine.connect(
            db=DATABASE_NAME,
            host=mongo_hostname,
            port=mongo_port,
            serverSelectionTimeoutMS=30)
    except mongoengine.ConnectionFailure as e:
        logger.error("Failed to connect to Mongodb", exc_info=e)
        raise e

    #
    # Force check to make sure Mongo is alive
    #
    try:
        server = database.server_info()
    except pymongo.errors.ServerSelectionTimeoutError as e:
        logger.error("Mongodb is offline", exc_info=e)
        raise e

    obj = RosWrapper(node)
    
    app = flask.Flask(__name__, template_folder='/map_manager/map_manager/http_utils')

    app.logger.setLevel(logging.INFO)
    for handler in handlers:
        app.logger.addHandler(handler)

    app.register_blueprint(map_api)

    #rclpy.spin(node) #Explicit spin for ros2 reasons

    app.run(host='0.0.0.0', port=RESOURCE_PORT)
    #rclpy.spin(node) #Thisone doesnt get called 
