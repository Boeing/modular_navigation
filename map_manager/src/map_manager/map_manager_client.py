#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

from map_manager.msg import Map as MapMsg
from map_manager.msg import Markers as MarkersMsg
from map_manager.msg import Zones as ZonesMsg
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


class MapManagerClient(object):
    def __init__(self, *args, **kwargs):
        self.add_map_srv = rospy.ServiceProxy(name='map_manager/add_map', service_class=AddMap)
        self.add_marker_srv = rospy.ServiceProxy(name='map_manager/add_marker', service_class=AddMarker)
        self.add_zone_srv = rospy.ServiceProxy(name='map_manager/add_zone', service_class=AddZone)

        self.delete_map_srv = rospy.ServiceProxy(name='map_manager/delete_map', service_class=DeleteMap)
        self.delete_marker_srv = rospy.ServiceProxy(name='map_manager/delete_marker', service_class=DeleteMarker)
        self.delete_zone_srv = rospy.ServiceProxy(name='map_manager/delete_zone', service_class=DeleteZone)

        self.get_map_srv = rospy.ServiceProxy(name='map_manager/get_map', service_class=GetMap)
        self.get_marker_srv = rospy.ServiceProxy(name='map_manager/get_marker', service_class=GetMap)
        self.get_occupancy_grid_srv = rospy.ServiceProxy('map_manager/get_occupancy_grid', GetOccupancyGrid)
        self.get_zone_srv = rospy.ServiceProxy(name='map_manager/get_zone', service_class=GetZone)

        self.list_maps_srv = rospy.ServiceProxy(name='map_manager/list_maps', service_class=ListMaps)
        self.list_markers_srv = rospy.ServiceProxy(name='map_manager/list_markers', service_class=ListMarkers)
        self.list_zones_srv = rospy.ServiceProxy(name='map_manager/list_zones', service_class=ListZones)

        self.set_active_map_srv = rospy.ServiceProxy(name='map_manager/set_active_map', service_class=SetActiveMap)

        self.update_map_srv = rospy.ServiceProxy(name='map_manager/update_map', service_class=UpdateMap)
        self.update_marker_srv = rospy.ServiceProxy(name='map_manager/update_marker', service_class=UpdateMarker)
        self.update_zone_srv = rospy.ServiceProxy(name='map_manager/update_zone', service_class=UpdateZone)

        self.add_map_srv.wait_for_service(timeout=10)
        self.add_marker_srv.wait_for_service(timeout=10)
        self.add_zone_srv.wait_for_service(timeout=10)

        self.delete_map_srv.wait_for_service(timeout=10)
        self.delete_marker_srv.wait_for_service(timeout=10)
        self.delete_zone_srv.wait_for_service(timeout=10)

        self.get_map_srv.wait_for_service(timeout=10)
        self.get_marker_srv.wait_for_service(timeout=10)
        self.get_occupancy_grid_srv.wait_for_service(timeout=10)
        self.get_zone_srv.wait_for_service(timeout=10)

        self.list_maps_srv.wait_for_service(timeout=10)
        self.list_markers_srv.wait_for_service(timeout=10)
        self.list_zones_srv.wait_for_service(timeout=10)

        self.set_active_map_srv.wait_for_service(timeout=10)

        self.update_map_srv.wait_for_service(timeout=10)
        self.update_marker_srv.wait_for_service(timeout=10)
        self.update_zone_srv.wait_for_service(timeout=10)

    def add_map_from_file(self, occupancy_grid_msg_file, map_name):
        # type: (str, str) -> None

        # Load test map from serialized file
        map_msg = OccupancyGrid()
        with open(occupancy_grid_msg_file, 'rb') as f:
            map_msg.deserialize(f.read())

        rospy.loginfo('Creating Map...')
        resp = self.add_map_srv.call(AddMapRequest(name=map_name, map=map_msg))
        assert isinstance(resp, AddMapResponse)
        if not resp.success:
            raise Exception('Failed to add Map: {}'.format(resp.message))
