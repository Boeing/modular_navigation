#!/usr/bin/env python

import logging
import flask
import mongoengine
import rospy.impl.rosout

from map_manager.config import DATABASE_NAME, RESOURCE_PORT
from map_manager.ros_wrapper import RosWrapper
from map_manager.http.routes import map_api
logger = logging.getLogger(__name__)

name = 'map_manager'

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

    mongo_hostname = rospy.get_param('~mongo_hostname', 'localhost')
    mongo_port = rospy.get_param('~mongo_port', 27017)

    logger.info('Starting Map Manager...')

    #
    # Connect to the db
    #
    logger.info('Connecting to {}:{}'.format(mongo_hostname, mongo_port))
    database = mongoengine.connect(db=DATABASE_NAME, host=mongo_hostname, port=mongo_port)

    obj = RosWrapper()
    app = flask.Flask(__name__, template_folder='src/map_manager/http')

    app.logger.setLevel(logging.INFO)
    for handler in handlers:
        app.logger.addHandler(handler)

    app.register_blueprint(map_api)

    app.run(host='0.0.0.0', port=RESOURCE_PORT)
