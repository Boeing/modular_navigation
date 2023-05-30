#!/usr/bin/env python3

import struct
import unittest
from io import BytesIO

import matplotlib
import rospy
# import rclpy
import rostest
from geometry_msgs.msg import Point, Pose, Quaternion
from map_manager.msg import MapInfo
from map_manager.srv import (AddMap, AddMapRequest, AddMapResponse,
                             SetActiveMap, SetActiveMapRequest,
                             SetActiveMapResponse)
from nav_msgs.msg import MapMetaData
from PIL import Image, ImageDraw
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8MultiArray

from map_manager.msgs import MapInfo
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from map_manager.srv import SetActiveMap, SetActiveMapRequest, SetActiveMapResponse


class TestMapManager(unittest.TestCase):

    def test_add_map_and_set_active(self):
        im = Image.new('L', (800, 800))
        d = ImageDraw.Draw(im)
        d.rectangle(((100, 100), (600, 600)), fill=100)
        d.rectangle(((300, 300), (350, 350)), fill=60)
        d.rectangle(((200, 200), (400, 400)), fill=0)
        b = BytesIO()
        im.save(fp=b, format='PNG')

        png_map_bytes = b.getvalue()

        map_meta = MapMetaData(
            map_load_time=rospy.Time(secs=0),
            resolution=0.02,
            width=800,
            height=800,
            origin=Pose(
                position=Point(x=-10., y=-10., z=0.),
                orientation=Quaternion(x=0., y=0., z=0., w=1.)
            )
        )
        map_name = 'test_map'
        add_map_srv = rospy.ServiceProxy('/map_manager/add_map', AddMap)
        add_map_srv.wait_for_service(timeout=10)
        request = AddMapRequest(
            map_info=MapInfo(name=map_name, meta_data=map_meta),
            occupancy_grid=CompressedImage(
                format='png',
                data=png_map_bytes
            ),
            pbstream=[1, 2, 3, 4]
        )
        rospy.loginfo('Adding a Map')
        response = add_map_srv.call(request)
        self.assertIsInstance(response, AddMapResponse)
        self.assertTrue(response.success, msg=response.message)

        set_map_srv = rospy.ServiceProxy('/map_manager/set_active_map', SetActiveMap)
        set_map_srv.wait_for_service(timeout=10)
        rospy.loginfo('Setting active map')
        response = set_map_srv.call(SetActiveMapRequest(map_name=map_name))
        self.assertIsInstance(response, SetActiveMapResponse)
        self.assertTrue(response.success, msg=response.message)

        md: UInt8MultiArray = rospy.wait_for_message('/map_manager/pbstream', UInt8MultiArray, timeout=10)
        md_d = struct.unpack('<%sb' % len(md.data), md.data)
        self.assertTrue(all(int(a) == int(b) for a, b in zip(md_d, request.pbstream)))


if __name__ == '__main__':
    rospy.init_node('TestMapManager')

    matplotlib.use('Agg')
    rostest.rosrun('map_manager', 'test_manager', TestMapManager)
