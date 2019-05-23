#!/usr/bin/env python

import copy
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Pose, Polygon, Point32, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from map_manager.srv import AddMarker, AddMarkerRequest, AddMarkerResponse
from map_manager.srv import AddZone, AddZoneRequest, AddZoneResponse
from map_manager.srv import DeleteMap, DeleteMapRequest, DeleteMapResponse
from map_manager.srv import DeleteMarker, DeleteMarkerRequest, DeleteMarkerResponse
from map_manager.srv import DeleteZone, DeleteZoneRequest, DeleteZoneResponse
from map_manager.srv import GetMap, GetMapRequest, GetMapResponse
from map_manager.srv import GetMarker, GetMarkerRequest, GetMarkerResponse
from map_manager.srv import GetOccupancyGrid, GetOccupancyGridRequest, GetOccupancyGridResponse
from map_manager.srv import GetZone, GetZoneRequest, GetZoneResponse
from map_manager.srv import ListMaps, ListMapsRequest, ListMapsResponse
from map_manager.srv import ListMarkers, ListMarkersRequest, ListMarkersResponse
from map_manager.srv import ListZones, ListZonesRequest, ListZonesResponse
from map_manager.srv import SetActiveMap, SetActiveMapRequest, SetActiveMapResponse
from map_manager.srv import UpdateMap, UpdateMapResponse, UpdateMapRequest
from map_manager.srv import UpdateMarker, UpdateMarkerResponse, UpdateMarkerRequest
from map_manager.srv import UpdateZone, UpdateZoneResponse, UpdateZoneRequest

from map_manager.msg import Zone, Zones, Markers

from map_manager.map_manager_client import MapManagerClient


class TopicBuffer(object):
    def __init__(self, name, data_class):
        self.__conditional = threading.Condition()
        self.__buffer = list()
        self.__expected_size = 0
        self.__sub = rospy.Subscriber(
            name=name,
            data_class=data_class,
            callback=self.__callback
        )

    def __callback(self, msg):
        with self.__conditional:
            self.__buffer.append(msg)
            self.__conditional.notify_all()

    def get_buffer(self):
        return copy.deepcopy(self.__buffer)

    def pop(self):
        with self.__conditional:
            item = self.__buffer.pop()
            self.__expected_size -= 1
            return item

    def clear(self):
        with self.__conditional:
            self.__expected_size = 0
            del self.__buffer[:]

    def wait_for_messages(self, number=1, timeout=5):
        self.__expected_size += number
        if len(self.__buffer) < self.__expected_size:
            with self.__conditional:
                self.__conditional.wait(timeout=timeout)
        return len(self.__buffer) >= self.__expected_size


class TestMapManager(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.__client = MapManagerClient()

        cls.__map_buffer = TopicBuffer(name='map_manager/occupancy_grid', data_class=OccupancyGrid)
        cls.__zones_buffer = TopicBuffer(name='map_manager/zones', data_class=Zones)
        cls.__markers_buffer = TopicBuffer(name='map_manager/markers', data_class=Markers)

    def setUp(self):
        self.__map_buffer.clear()
        self.__zones_buffer.clear()
        self.__markers_buffer.clear()

        # Create a trivial world
        rospy.loginfo('Creating a test World')
        self.__map_msg = OccupancyGrid(
            header=Header(seq=0, stamp=rospy.Time.now(), frame_id='map'),
            info=MapMetaData(
                map_load_time=rospy.Time(0),
                resolution=0.1,
                width=8,
                height=8,
                origin=Pose(
                    position=Point(x=1, y=2, z=3),
                    orientation=Quaternion(x=0, y=0, z=0, w=1)
                )
            ),
            data=64 * (0,)
        )
        self.__map_name = 'test_map'
        resp = self.__client.add_map_srv.call(
            AddMapRequest(
                name=self.__map_name,
                map=self.__map_msg
            )
        )
        self.assertIsInstance(resp, AddMapResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Wait for messages to arrive
        self.assertTrue(self.__map_buffer.wait_for_messages(number=1))
        self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))

    def tearDown(self):
        resp = self.__client.delete_map_srv.call(
            DeleteMapRequest(
                map_name=self.__map_name
            )
        )
        self.assertIsInstance(resp, DeleteMapResponse)
        self.assertTrue(resp.success, msg=resp.message)

    def test_map_data(self):
        # Clear map buffer
        self.__map_buffer.clear()

        # Load the world
        rospy.loginfo('Loading the new World')
        resp = self.__client.set_active_map_srv.call(
            SetActiveMapRequest(map_name=self.__map_name)
        )
        self.assertIsInstance(resp, SetActiveMapResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Wait for map data
        self.assertTrue(self.__map_buffer.wait_for_messages(number=1))
        map_msg = self.__map_buffer.pop()

        # Validate map data
        map_msg.info.resolution = round(map_msg.info.resolution, 2)

        self.assertEqual(map_msg.info.map_load_time, self.__map_msg.info.map_load_time)
        self.assertEqual(map_msg.info.resolution, self.__map_msg.info.resolution)
        self.assertEqual(map_msg.info.width, self.__map_msg.info.width)
        self.assertEqual(map_msg.info.height, self.__map_msg.info.height)
        self.assertEqual(map_msg.info.origin, self.__map_msg.info.origin)

        self.assertEqual(map_msg.info, self.__map_msg.info)
        self.assertEqual(map_msg.data, self.__map_msg.data)

    def test_list_maps(self):
        # List the maps
        rospy.loginfo('Listing Map data')
        list_response = self.__client.list_maps_srv.call(
            ListMapsRequest()
        )
        self.assertIsInstance(list_response, ListMapsResponse)
        self.assertTrue(list_response.success, msg=list_response.message)
        self.assertTrue(len(list_response.maps) == 1)

    def test_zones(self):
        # Build polygon
        poly = Polygon()
        poly.points.append(Point32(x=0, y=0, z=0))
        poly.points.append(Point32(x=1, y=0, z=0))
        poly.points.append(Point32(x=1, y=1, z=0))
        poly.points.append(Point32(x=0, y=1, z=0))

        # Reset msg buffer
        self.__zones_buffer.clear()

        # Create a new Zone
        rospy.loginfo('Creating a new Zone')
        resp = self.__client.add_zone_srv.call(
            AddZoneRequest(
                map_name=self.__map_name,
                name='z1',
                zone_type=Zone.EXCLUSION_ZONE,
                polygon=poly
            )
        )
        self.assertIsInstance(resp, AddZoneResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Validate polygon
        self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))
        zones_msg = self.__zones_buffer.pop()
        self.assertTrue(len(zones_msg.zones) == 1)
        self.assertEqual(zones_msg.zones[0].name, 'z1')
        self.assertEqual(zones_msg.zones[0].zone_type, Zone.EXCLUSION_ZONE)
        self.assertEqual(zones_msg.zones[0].polygon, poly)

        # Update the cs
        poly.points.append(Point32(x=0, y=0.5, z=0))
        resp = self.__client.update_zone_srv.call(
            UpdateZoneRequest(
                map_name=self.__map_name,
                name='z1',
                zone_type=Zone.EXCLUSION_ZONE,
                polygon=poly
            )
        )
        self.assertIsInstance(resp, UpdateZoneResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Validate polygon
        self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))
        zones_msg = self.__zones_buffer.pop()
        self.assertTrue(len(zones_msg.zones) == 1)
        self.assertEqual(zones_msg.zones[0].name, 'z1')
        self.assertEqual(zones_msg.zones[0].zone_type, Zone.EXCLUSION_ZONE)
        self.assertEqual(zones_msg.zones[0].polygon, poly)

        # Delete zone
        resp = self.__client.delete_zone_srv.call(
            DeleteZoneRequest(map_name=self.__map_name, name='z1')
        )
        self.assertIsInstance(resp, DeleteZoneResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Validate zones
        self.assertTrue(self.__zones_buffer.wait_for_messages(number=1))
        zones_msg = self.__zones_buffer.pop()
        self.assertTrue(len(zones_msg.zones) == 0)

    def test_markers(self):
        # Build pose
        pose = Pose(
            position=Point(x=1, y=2, z=3),
            orientation=Quaternion(w=1)
        )

        # Reset msg buffer
        self.__markers_buffer.clear()

        # Create a new Marker
        rospy.loginfo('Creating a new Marker')
        resp = self.__client.add_marker_srv.call(
            AddMarkerRequest(
                map_name=self.__map_name,
                name='m1',
                marker_type=1,
                pose=pose
            )
        )
        self.assertIsInstance(resp, AddMarkerResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Validate
        self.assertTrue(self.__markers_buffer.wait_for_messages(number=1))
        markers_msg = self.__markers_buffer.pop()  # type: Markers
        self.assertTrue(len(markers_msg.markers) == 1)
        self.assertEqual(markers_msg.markers[0].name, 'm1')
        self.assertEqual(markers_msg.markers[0].marker_type, 1)
        self.assertEqual(markers_msg.markers[0].pose, pose)

        # Update the Marker
        pose.position.x = 10
        resp = self.__client.update_marker_srv.call(
            UpdateMarkerRequest(
                map_name=self.__map_name,
                name='m1',
                marker_type=2,
                pose=pose
            )
        )
        self.assertIsInstance(resp, UpdateMarkerResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Validate
        self.assertTrue(self.__markers_buffer.wait_for_messages(number=1))
        markers_msg = self.__markers_buffer.pop()  # type: Markers
        self.assertTrue(len(markers_msg.markers) == 1)
        self.assertEqual(markers_msg.markers[0].name, 'm1')
        self.assertEqual(markers_msg.markers[0].marker_type, 2)
        self.assertEqual(markers_msg.markers[0].pose, pose)

        # Delete Marker
        resp = self.__client.delete_marker_srv.call(
            DeleteMarkerRequest(map_name=self.__map_name, name='m1')
        )
        self.assertIsInstance(resp, DeleteMarkerResponse)
        self.assertTrue(resp.success, msg=resp.message)

        # Validate Marker
        self.assertTrue(self.__markers_buffer.wait_for_messages(number=1))
        markers_msg = self.__markers_buffer.pop()  # type: Markers
        self.assertTrue(len(markers_msg.markers) == 0)

    def test_get_map_data(self):
        # Get the worlds map
        rospy.loginfo('Loading the Map')
        resp = self.__client.get_occupancy_grid_srv.call(
            GetOccupancyGridRequest(map_name=self.__map_name)
        )
        self.assertIsInstance(resp, GetOccupancyGridResponse)
        self.assertTrue(resp.success, msg=resp.message)

        map_msg = resp.map

        # Validate map data
        map_msg.info.resolution = round(map_msg.info.resolution, 2)

        self.assertEqual(map_msg.info.map_load_time, self.__map_msg.info.map_load_time)
        self.assertEqual(map_msg.info.resolution, self.__map_msg.info.resolution)
        self.assertEqual(map_msg.info.width, self.__map_msg.info.width)
        self.assertEqual(map_msg.info.height, self.__map_msg.info.height)
        self.assertEqual(map_msg.info.origin, self.__map_msg.info.origin)

        self.assertEqual(map_msg.info, self.__map_msg.info)
        self.assertEqual(map_msg.data, self.__map_msg.data)


if __name__ == '__main__':
    rospy.init_node('TestMapManager')

    rostest.rosrun('map_manager', 'test_manager', TestMapManager)
