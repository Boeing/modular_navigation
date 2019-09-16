#!/usr/bin/env python

import struct
import unittest
from PIL import Image, ImageDraw
from io import BytesIO

import rospy
import rostest
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8MultiArray

from hd_map.msg import Map, MapInfo, Marker
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from map_manager.srv import SetActiveMap, SetActiveMapRequest, SetActiveMapResponse


class TestMapManager(unittest.TestCase):

    # @classmethod
    # def setUpClass(cls):
    # cls.__map_buffer = TopicBuffer(name='map_manager/occupancy_grid', data_class=OccupancyGrid)
    # cls.__zones_buffer = TopicBuffer(name='map_manager/zones', data_class=Zones)
    # cls.__markers_buffer = TopicBuffer(name='map_manager/markers', data_class=Markers)

    # def setUp(self):
    #     self.__map_buffer.clear()
    #     self.__zones_buffer.clear()
    #     self.__markers_buffer.clear()
    #
    #     # Create a trivial world
    #     rospy.loginfo('Creating a test World')
    #     self.__map_msg = OccupancyGrid(
    #         header=Header(seq=0, stamp=rospy.Time.now(), frame_id='map'),
    #         info=MapMetaData(
    #             map_load_time=rospy.Time(0),
    #             resolution=0.1,
    #             width=8,
    #             height=8,
    #             origin=Pose(
    #                 position=Point(x=1, y=2, z=3),
    #                 orientation=Quaternion(x=0, y=0, z=0, w=1)
    #             )
    #         ),
    #         data=64 * (0,)
    #     )
    #     self.__map_name = 'test_map'
    #     resp = self.__client.add_map_srv.call(
    #         AddMapRequest(
    #             name=self.__map_name,
    #             map=self.__map_msg
    #         )
    #     )
    #     self.assertIsInstance(resp, AddMapResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Wait for messages to arrive
    #     self.assertTrue(self.__map_buffer.wait_for_messages(number=1))
    #     self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))

    def test_add_map_and_set_active(self):
        im = Image.new('L', (800, 800))
        d = ImageDraw.Draw(im)
        d.rectangle(((100, 100), (600, 600)), fill=100)
        d.rectangle(((300, 300), (350, 350)), fill=60)
        d.rectangle(((200, 200), (400, 400)), fill=0)
        b = BytesIO()
        im.save(fp=b, format='PNG')

        png_map_bytes = b.getvalue()

        map_info = MapMetaData(
            map_load_time=rospy.Time(0),
            resolution=0.02,
            width=800,
            height=800,
            origin=Pose(
                position=Point(x=-10, y=-10, z=0),
                orientation=Quaternion(x=0, y=0, z=0, w=1)
            )
        )
        map_name = 'test_map'
        add_map_srv = rospy.ServiceProxy('/map_manager/add_map', AddMap)
        add_map_srv.wait_for_service(timeout=10)
        request = AddMapRequest(
            map=Map(
                info=MapInfo(name=map_name, meta_data=map_info),
                markers=[
                    Marker(name='marker_{}'.format(i), pose=Pose(position=Point(x=i, y=i), orientation=Quaternion(w=1)))
                    for i in range(10)
                ]
            ),
            occupancy_grid=CompressedImage(
                format='png',
                data=png_map_bytes
            ),
            map_data=[1, 2, 3, 4]
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

        md = rospy.wait_for_message('/map_manager/map_data', UInt8MultiArray, timeout=10)  # type: UInt8MultiArray
        md_d = struct.unpack('<%sb' % len(md.data), md.data)
        self.assertTrue(all(int(a) == int(b) for a, b in zip(md_d, request.map_data)))

        # og = rospy.wait_for_message('/map_manager/occupancy_grid', OccupancyGrid, timeout=10)  # type: OccupancyGrid

        # Wait for messages to arrive
        # self.assertTrue(self.__map_buffer.wait_for_messages(number=1))
        # self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))

    # def tearDown(self):
    #     resp = self.__client.delete_map_srv.call(
    #         DeleteMapRequest(
    #             map_name=self.__map_name
    #         )
    #     )
    #     self.assertIsInstance(resp, DeleteMapResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    # def test_map_data(self):
    #     # Clear map buffer
    #     self.__map_buffer.clear()
    #
    #     # Load the world
    #     rospy.loginfo('Loading the new World')
    #     resp = self.__client.set_active_map_srv.call(
    #         SetActiveMapRequest(map_name=self.__map_name)
    #     )
    #     self.assertIsInstance(resp, SetActiveMapResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Wait for map data
    #     self.assertTrue(self.__map_buffer.wait_for_messages(number=1))
    #     map_msg = self.__map_buffer.pop()
    #
    #     # Validate map data
    #     map_msg.info.resolution = round(map_msg.info.resolution, 2)
    #
    #     self.assertEqual(map_msg.info.map_load_time, self.__map_msg.info.map_load_time)
    #     self.assertEqual(map_msg.info.resolution, self.__map_msg.info.resolution)
    #     self.assertEqual(map_msg.info.width, self.__map_msg.info.width)
    #     self.assertEqual(map_msg.info.height, self.__map_msg.info.height)
    #     self.assertEqual(map_msg.info.origin, self.__map_msg.info.origin)
    #
    #     self.assertEqual(map_msg.info, self.__map_msg.info)
    #     self.assertEqual(map_msg.data, self.__map_msg.data)
    #
    # def test_list_maps(self):
    #     # List the maps
    #     rospy.loginfo('Listing Map data')
    #     list_response = self.__client.list_maps_srv.call(
    #         ListMapsRequest()
    #     )
    #     self.assertIsInstance(list_response, ListMapsResponse)
    #     self.assertTrue(list_response.success, msg=list_response.message)
    #     self.assertTrue(len(list_response.maps) == 1)

    # def test_zones(self):
    #     # Build polygon
    #     poly = Polygon()
    #     poly.points.append(Point32(x=0, y=0, z=0))
    #     poly.points.append(Point32(x=1, y=0, z=0))
    #     poly.points.append(Point32(x=1, y=1, z=0))
    #     poly.points.append(Point32(x=0, y=1, z=0))
    #
    #     # Reset msg buffer
    #     self.__zones_buffer.clear()
    #
    #     # Create a new Zone
    #     rospy.loginfo('Creating a new Zone')
    #     resp = self.__client.add_zone_srv.call(
    #         AddZoneRequest(
    #             map_name=self.__map_name,
    #             name='z1',
    #             zone_type=Zone.EXCLUSION_ZONE,
    #             polygon=poly
    #         )
    #     )
    #     self.assertIsInstance(resp, AddZoneResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Validate polygon
    #     self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))
    #     zones_msg = self.__zones_buffer.pop()
    #     self.assertTrue(len(zones_msg.zones) == 1)
    #     self.assertEqual(zones_msg.zones[0].name, 'z1')
    #     self.assertEqual(zones_msg.zones[0].zone_type, Zone.EXCLUSION_ZONE)
    #     self.assertEqual(zones_msg.zones[0].polygon, poly)
    #
    #     # Update the cs
    #     poly.points.append(Point32(x=0, y=0.5, z=0))
    #     resp = self.__client.update_zone_srv.call(
    #         UpdateZoneRequest(
    #             map_name=self.__map_name,
    #             name='z1',
    #             zone_type=Zone.EXCLUSION_ZONE,
    #             polygon=poly
    #         )
    #     )
    #     self.assertIsInstance(resp, UpdateZoneResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Validate polygon
    #     self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))
    #     zones_msg = self.__zones_buffer.pop()
    #     self.assertTrue(len(zones_msg.zones) == 1)
    #     self.assertEqual(zones_msg.zones[0].name, 'z1')
    #     self.assertEqual(zones_msg.zones[0].zone_type, Zone.EXCLUSION_ZONE)
    #     self.assertEqual(zones_msg.zones[0].polygon, poly)
    #
    #     # Delete zone
    #     resp = self.__client.delete_zone_srv.call(
    #         DeleteZoneRequest(map_name=self.__map_name, name='z1')
    #     )
    #     self.assertIsInstance(resp, DeleteZoneResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Validate zones
    #     self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))
    #     zones_msg = self.__zones_buffer.pop()
    #     self.assertTrue(len(zones_msg.zones) == 0)

    # def test_markers(self):
    #     # Build pose
    #     pose = Pose(
    #         position=Point(x=1, y=2, z=3),
    #         orientation=Quaternion(w=1)
    #     )
    #
    #     # Reset msg buffer
    #     self.__markers_buffer.clear()
    #
    #     # Create a new Marker
    #     rospy.loginfo('Creating a new Marker')
    #     resp = self.__client.add_marker_srv.call(
    #         AddMarkerRequest(
    #             map_name=self.__map_name,
    #             name='m1',
    #             marker_type=1,
    #             pose=pose
    #         )
    #     )
    #     self.assertIsInstance(resp, AddMarkerResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Validate
    #     self.assertTrue(self.__markers_buffer.wait_for_messages(number=1))
    #     markers_msg = self.__markers_buffer.pop()  # type: Markers
    #     self.assertTrue(len(markers_msg.markers) == 1)
    #     self.assertEqual(markers_msg.markers[0].name, 'm1')
    #     self.assertEqual(markers_msg.markers[0].marker_type, 1)
    #     self.assertEqual(markers_msg.markers[0].pose, pose)
    #
    #     # Update the Marker
    #     pose.position.x = 10
    #     resp = self.__client.update_marker_srv.call(
    #         UpdateMarkerRequest(
    #             map_name=self.__map_name,
    #             name='m1',
    #             marker_type=2,
    #             pose=pose
    #         )
    #     )
    #     self.assertIsInstance(resp, UpdateMarkerResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Validate
    #     self.assertTrue(self.__markers_buffer.wait_for_messages(number=1))
    #     markers_msg = self.__markers_buffer.pop()  # type: Markers
    #     self.assertTrue(len(markers_msg.markers) == 1)
    #     self.assertEqual(markers_msg.markers[0].name, 'm1')
    #     self.assertEqual(markers_msg.markers[0].marker_type, 2)
    #     self.assertEqual(markers_msg.markers[0].pose, pose)
    #
    #     # Delete Marker
    #     resp = self.__client.delete_marker_srv.call(
    #         DeleteMarkerRequest(map_name=self.__map_name, name='m1')
    #     )
    #     self.assertIsInstance(resp, DeleteMarkerResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     # Validate Marker
    #     self.assertTrue(self.__markers_buffer.wait_for_messages(number=1))
    #     markers_msg = self.__markers_buffer.pop()  # type: Markers
    #     self.assertTrue(len(markers_msg.markers) == 0)

    # def test_get_map_data(self):
    #     # Get the worlds map
    #     rospy.loginfo('Loading the Map')
    #     resp = self.__client.get_occupancy_grid_srv.call(
    #         GetOccupancyGridRequest(map_name=self.__map_name)
    #     )
    #     self.assertIsInstance(resp, GetOccupancyGridResponse)
    #     self.assertTrue(resp.success, msg=resp.message)
    #
    #     map_msg = resp.map
    #
    #     # Validate map data
    #     map_msg.info.resolution = round(map_msg.info.resolution, 2)
    #
    #     self.assertEqual(map_msg.info.map_load_time, self.__map_msg.info.map_load_time)
    #     self.assertEqual(map_msg.info.resolution, self.__map_msg.info.resolution)
    #     self.assertEqual(map_msg.info.width, self.__map_msg.info.width)
    #     self.assertEqual(map_msg.info.height, self.__map_msg.info.height)
    #     self.assertEqual(map_msg.info.origin, self.__map_msg.info.origin)
    #
    #     self.assertEqual(map_msg.info, self.__map_msg.info)
    #     self.assertEqual(map_msg.data, self.__map_msg.data)


if __name__ == '__main__':
    rospy.init_node('TestMapManager')

    rostest.rosrun('map_manager', 'test_manager', TestMapManager)
