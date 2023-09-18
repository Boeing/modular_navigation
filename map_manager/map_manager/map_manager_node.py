#!/usr/bin/env python3

import logging

import flask
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from map_manager.ros_wrapper import RosWrapper
from map_manager.http_utils.routes import map_api
from map_manager.config import RESOURCE_PORT
from threading import Thread


def main(args=None):
    rclpy.init(args=args)

    def spin_srv(executor):
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass

    node = rclpy.create_node('map_manager', start_parameter_services=False)
    logger = node.get_logger()

    # Spin in a separate thread
    srv_executor = MultiThreadedExecutor()
    srv_executor.add_node(node)
    srv_thread = Thread(target=spin_srv, args=(srv_executor,), daemon=True)
    srv_thread.start()

    logger.info('Starting Map Manager...')

    map_manager = RosWrapper(node)

    app = flask.Flask(__name__, template_folder='/map_manager/map_manager/http_utils')
    server_name = '0.0.0.0:' + str(RESOURCE_PORT)

    logger.info('Running MapManager at address:' + server_name)
    app.config['SERVER_NAME'] = server_name
    app.logger.setLevel(logging.INFO)

    app.register_blueprint(map_api)

    server_thread = threading.Thread(target=app.run, daemon=True)
    server_thread.start()

    rate = node.create_rate(1)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    logger.info('Shutting down...')
    rate.destroy()
    srv_executor.shutdown()
    node.destroy_node()  # destroy the node explicity (optional)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
