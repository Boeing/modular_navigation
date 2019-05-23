import logging
import traceback
import typing
from functools import wraps

import geometry_msgs.msg
import numpy
import rospy
import std_msgs.msg
from PIL import Image
from mongoengine import DoesNotExist, ValidationError
from pymongo import MongoClient
from tf2_msgs.msg import TFMessage

from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import MapMetaData as MapMetaDataMsg

from std_msgs.msg import String

from map_manager.documents import Map, Zone, Marker, OccupancyGrid, Pose, Point, Quaternion
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

logger = logging.getLogger(__name__)


def exception_wrapper(return_cls):
    def wrapper(func):
        @wraps(func)
        def func_wrapper(self, msg, *args, **kwargs):
            try:
                return func(self, msg, *args, **kwargs)
            except ValidationError as e:
                logger.error('exception_wrapper: ValidationError: {}'.format(str(e)))
                return return_cls(success=False, message='object_id is not valid: {}'.format(str(e)))
            except DoesNotExist as e:
                logger.error('exception_wrapper: object does not exist: {}'.format(str(e)))
                return return_cls(success=False, message='Object does not exist: {}'.format(str(e)))
            except Exception as e:
                msg = 'exception_wrapper: exception: {}: {}'.format(str(e), traceback.format_exc())
                logger.error(msg)
                return return_cls(success=False, message=msg)

        return func_wrapper

    return wrapper


class RosWrapper(object):
    def __init__(self):

        #
        # State
        #
        self.__map_name = None

        self.__add_map = rospy.Service('~add_map', AddMap, handler=self.__add_map_cb)
        self.__add_marker = rospy.Service('~add_marker', AddMarker, handler=self.__add_marker_cb)
        self.__add_zone = rospy.Service('~add_zone', AddZone, handler=self.__add_zone_cb)

        self.__delete_map = rospy.Service('~delete_map', DeleteMap, handler=self.__delete_map_cb)
        self.__delete_marker = rospy.Service('~delete_marker', DeleteMarker, handler=self.__delete_marker_cb)
        self.__delete_zone = rospy.Service('~delete_zone', DeleteZone, handler=self.__delete_zone_cb)

        self.__get_map = rospy.Service('~get_map', GetMap, handler=self.__get_map_cb)
        self.__get_marker = rospy.Service('~get_marker', GetMarker, handler=self.__get_marker_cb)
        self.__get_og = rospy.Service('~get_occupancy_grid', GetOccupancyGrid, handler=self.__get_occupancy_grid_cb)
        self.__get_zone = rospy.Service('~get_zone', GetZone, handler=self.__get_zone_cb)

        self.__list_map = rospy.Service('~list_maps', ListMaps, handler=self.__list_maps_cb)
        self.__list_marker = rospy.Service('~list_markers', ListMarkers, handler=self.__list_markers_cb)
        self.__list_zone = rospy.Service('~list_zones', ListZones, handler=self.__list_zones_cb)

        self.__list_zone = rospy.Service('~set_active_map', SetActiveMap, handler=self.__set_active_map_cb)

        self.__update_map = rospy.Service('~update_map', UpdateMap, handler=self.__update_map_cb)
        self.__update_marker = rospy.Service('~update_marker', UpdateMarker, handler=self.__update_marker_cb)
        self.__update_zone = rospy.Service('~update_zone', UpdateZone, handler=self.__update_zone_cb)

        # Initialise a unit world->map transform
        self.__static_tf_pub = rospy.Publisher("/tf_static", TFMessage, queue_size=100, latch=True)
        self.__static_tf_pub.publish(
            TFMessage(
                transforms=[
                    geometry_msgs.msg.TransformStamped(
                        header=std_msgs.msg.Header(frame_id='world', stamp=rospy.Time(0)),
                        child_frame_id='map',
                        transform=geometry_msgs.msg.Transform(
                            translation=geometry_msgs.msg.Vector3(0, 0, 0),
                            rotation=geometry_msgs.msg.Quaternion(0, 0, 0, 1)
                        )
                    )
                ]
            )
        )

        #
        # Publishers
        #
        self.__active_map_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__og_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__map_meta_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__markers_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__zones_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__map_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__init_publishers()

        # Load the most recently modified Map by default
        map_query = Map.objects.order_by('-modified')
        if map_query.count():
            map_obj = map_query.first()
            self.__load_map(map_name=str(map_obj.name))
        else:
            self.__active_map_pub.publish(String(data=''))

        logger.info('Successfully started')

    def __init_publishers(self):
        logger.info('Initialising publishers')

        self.__active_map_pub = rospy.Publisher(
            name='~active_map',
            data_class=String,
            latch=True,
            queue_size=1000
        )
        self.__og_pub = rospy.Publisher(
            name='~occupancy_grid',
            data_class=OccupancyGridMsg,
            latch=True,
            queue_size=1000
        )
        self.__map_meta_pub = rospy.Publisher(
            name='~map_metadata',
            data_class=MapMetaDataMsg,
            latch=True,
            queue_size=1000
        )
        self.__markers_pub = rospy.Publisher(
            name='~markers',
            data_class=MarkersMsg,
            latch=True,
            queue_size=1000
        )
        self.__zones_pub = rospy.Publisher(
            name='~zones',
            data_class=ZonesMsg,
            latch=True,
            queue_size=1000
        )
        self.__map_pub = rospy.Publisher(
            name='~map',
            data_class=MapMsg,
            latch=True,
            queue_size=1000
        )

    #
    # ADD callbacks
    #

    @exception_wrapper(AddMapResponse)
    def __add_map_cb(self, req):
        # type: (AddMapRequest) -> AddMapResponse
        logger.info('Request to add a new Map: {}'.format(req.name))

        if len(req.map.data) <= 0:
            return AddMapResponse(
                success=False,
                message='No map data'
            )

        og_map = OccupancyGrid()
        og_map.width = req.map.info.width
        og_map.height = req.map.info.height
        og_map.resolution = req.map.info.resolution
        og_map.origin = Pose(
            position=Point(
                x=req.map.info.origin.position.x,
                y=req.map.info.origin.position.y,
                z=req.map.info.origin.position.z),
            quaternion=Quaternion(
                x=req.map.info.origin.orientation.x,
                y=req.map.info.origin.orientation.y,
                z=req.map.info.origin.orientation.z,
                w=req.map.info.origin.orientation.w
            )
        )
        im = Image.fromarray(numpy.asarray(req.map.data, dtype=numpy.uint8)
                             .reshape(req.map.info.height, req.map.info.width))
        og_map.image.new_file()
        im.save(fp=og_map.image, format='PPM')
        og_map.image.close()

        obj = Map(
            name=req.name,
            description=req.description,
            map=og_map
        )
        obj.save()

        self.__load_map(map_name=req.name)

        return AddMapResponse(
            success=True
        )

    @exception_wrapper(AddMarkerResponse)
    def __add_marker_cb(self, req):
        # type: (AddMarkerRequest) -> AddMarkerResponse
        logger.info('Request to add a new Marker: {}'.format(req.name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            # First check if there is an existing Marker with this name
            if any([m.name == req.name for m in map_obj.markers]):
                return AddMarkerResponse(
                    success=False,
                    message='Marker {} already exists'.format(req.name)
                )

        except DoesNotExist:
            pass

        obj = Marker(
            name=req.name,
            description=req.description,
            marker_type=req.marker_type,
            pose=Pose(
                position=Point(
                    x=req.pose.position.x,
                    y=req.pose.position.y,
                    z=req.pose.position.z
                ),
                quaternion=Quaternion(
                    w=req.pose.orientation.w,
                    x=req.pose.orientation.x,
                    y=req.pose.orientation.y,
                    z=req.pose.orientation.z
                )
            )
        )
        map_obj.markers.append(obj)
        map_obj.save()

        if req.map_name == self.__map_name:
            self.__publish_markers()
            self.__publish_map()

        return AddMarkerResponse(
            success=True
        )

    @exception_wrapper(AddZoneResponse)
    def __add_zone_cb(self, req):
        # type: (AddZoneRequest) -> AddZoneResponse
        logger.info('Request to add a new Zone: {}'.format(req.name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            # First check if there is an existing Zone with this name
            if any([z.name == req.name for z in map_obj.zones]):
                return AddZoneResponse(
                    success=False,
                    message='Zone {} already exists'.format(req.name)
                )

        except DoesNotExist:
            pass

        obj = Zone(
            name=req.name,
            description=req.description,
            zone_type=req.zone_type,
            polygon=[Point(x=point.x, y=point.y, z=point.z) for point in req.polygon.points]
        )
        map_obj.zones.append(obj)
        map_obj.save()

        if req.map_name == self.__map_name:
            self.__publish_zones()
            self.__publish_map()

        return AddZoneResponse(
            success=True
        )

    #
    # DELETE callbacks
    #

    @exception_wrapper(DeleteMapResponse)
    def __delete_map_cb(self, req):
        # type: (DeleteMapRequest) -> DeleteMapResponse
        logger.info('Request to delete Map: {}'.format(req.map_name))

        obj = Map.objects(name=req.map_name).get()
        obj.delete()

        if req.map_name == self.__map_name:
            self.__init_publishers()
            self.__map_name = None

        return DeleteMapResponse(
            success=True
        )

    @exception_wrapper(DeleteMarkerResponse)
    def __delete_marker_cb(self, req):
        # type: (DeleteMarkerRequest) -> DeleteMarkerResponse
        logger.info('Request to delete Marker: {}'.format(req.name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            marker = next(m for m in map_obj.markers if m.name == req.name)  # type: Marker
        except StopIteration:
            return DeleteMarkerResponse(
                success=False,
                message='No Marker matching {}'.format(req.name)
            )

        map_obj.markers.remove(marker)
        map_obj.save()

        if req.map_name == self.__map_name:
            self.__publish_markers()
            self.__publish_map()

        return DeleteMarkerResponse(
            success=True
        )

    @exception_wrapper(DeleteZoneResponse)
    def __delete_zone_cb(self, req):
        # type: (DeleteZoneRequest) -> DeleteZoneResponse
        logger.info('Request to delete Zone: {}'.format(req.name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            zone = next(z for z in map_obj.zones if z.name == req.name)  # type: Zone
        except StopIteration:
            return DeleteZoneResponse(
                success=False,
                message='No Zone matching {}'.format(req.name)
            )

        map_obj.zones.remove(zone)
        map_obj.save()

        if req.map_name == self.__map_name:
            self.__publish_zones()
            self.__publish_map()

        return DeleteZoneResponse(
            success=True
        )

    #
    # GET callbacks
    #

    @exception_wrapper(GetOccupancyGridResponse)
    def __get_occupancy_grid_cb(self, req):
        # type: (GetOccupancyGridRequest) -> GetOccupancyGridResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return GetOccupancyGridResponse(
            map=map_obj.map.get_occupancy_grid_msg(),
            success=True
        )

    @exception_wrapper(GetMapResponse)
    def __get_map_cb(self, req):
        # type: (GetMapRequest) -> GetMapResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return GetMapResponse(
            map=map_obj.get_msg(),
            success=True
        )

    @exception_wrapper(GetMarkerResponse)
    def __get_marker_cb(self, req):
        # type: (GetMarkerRequest) -> GetMarkerResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            marker = next(m for m in map_obj.markers if m.name == req.name)  # type: Marker
        except StopIteration:
            return GetMarkerResponse(
                success=False,
                message='No Marker matching {}'.format(req.name)
            )

        return GetMarkerResponse(
            marker=marker.get_msg(),
            success=True
        )

    @exception_wrapper(GetZoneResponse)
    def __get_zone_cb(self, req):
        # type: (GetZoneRequest) -> GetZoneResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            zone = next(z for z in map_obj.zones if z.name == req.name)  # type: Zone
        except StopIteration:
            return GetZoneResponse(
                success=False,
                message='No Zone matching {}'.format(req.name)
            )

        return GetZoneResponse(
            zone=zone.get_msg(),
            success=True
        )

    #
    # LIST callbacks
    #

    @exception_wrapper(ListMapsResponse)
    def __list_maps_cb(self, _):
        # type: (ListMapsRequest) -> ListMapsResponse
        return ListMapsResponse(
            active_map=self.__map_name if self.__map_name else '',
            maps=[obj.get_msg() for obj in Map.objects],
            success=True
        )

    @exception_wrapper(ListMarkersResponse)
    def __list_markers_cb(self, req):
        # type: (ListMarkersRequest) -> ListMarkersResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return ListMarkersResponse(
            markers=[obj.get_msg() for obj in map_obj.markers],
            success=True
        )

    @exception_wrapper(ListZonesResponse)
    def __list_zones_cb(self, req):
        # type: (ListZonesRequest) -> ListZonesResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return ListZonesResponse(
            zones=[obj.get_msg() for obj in map_obj.zones],
            success=True
        )

    #
    # SET callbacks
    #

    @exception_wrapper(SetActiveMapResponse)
    def __set_active_map_cb(self, req):
        # type: (SetActiveMapRequest) -> SetActiveMapResponse
        logger.info('Request to set active Map: {}'.format(req.map_name))

        map_obj = Map.objects(name=req.map_name).get()
        self.__load_map(map_name=req.map_name)

        return SetActiveMapResponse(
            success=True
        )

    #
    # UPDATE callbacks
    #

    @exception_wrapper(UpdateMapResponse)
    def __update_map_cb(self, req):
        # type: (UpdateMapRequest) -> UpdateMapResponse
        logger.info('Request to update Map: {}'.format(req.map_name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        if len(req.map.data) <= 0:
            return UpdateMapResponse(
                success=False,
                message='No map data'
            )

        og_map = OccupancyGrid()
        og_map.width = req.map.info.width
        og_map.height = req.map.info.height
        og_map.resolution = req.map.info.resolution
        og_map.origin = Pose(
            position=Point(
                x=req.map.info.origin.position.x,
                y=req.map.info.origin.position.y,
                z=req.map.info.origin.position.z),
            quaternion=Quaternion(
                x=req.map.info.origin.orientation.x,
                y=req.map.info.origin.orientation.y,
                z=req.map.info.origin.orientation.z,
                w=req.map.info.origin.orientation.w
            )
        )
        im = Image.fromarray(numpy.asarray(req.map.data, dtype=numpy.uint8)
                             .reshape(req.map.info.height, req.map.info.width))
        og_map.image.new_file()
        im.save(fp=og_map.image, format='PPM')
        og_map.image.close()

        map_obj.description = req.description
        map_obj.map = og_map
        map_obj.save()

        if req.map_name == self.__map_name:
            self.__publish_map()

        return UpdateMapResponse(
            success=True
        )

    @exception_wrapper(UpdateMarkerResponse)
    def __update_marker_cb(self, req):
        # type: (UpdateMarkerRequest) -> UpdateMarkerResponse
        logger.info('Request to update Marker: {}'.format(req.name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            marker = next(m for m in map_obj.markers if m.name == req.name)  # type: Marker
        except StopIteration:
            return UpdateMarkerResponse(
                success=False,
                message='No Marker matching {}'.format(req.name)
            )

        marker.description = req.description
        marker.marker_type = req.marker_type
        marker.pose = Pose(
            position=Point(
                x=req.pose.position.x,
                y=req.pose.position.y,
                z=req.pose.position.z
            ),
            quaternion=Quaternion(
                w=req.pose.orientation.w,
                x=req.pose.orientation.x,
                y=req.pose.orientation.y,
                z=req.pose.orientation.z
            )
        )
        marker.save()

        if req.map_name == self.__map_name:
            self.__publish_markers()
            self.__publish_map()

        return UpdateMarkerResponse(
            success=True
        )

    @exception_wrapper(UpdateZoneResponse)
    def __update_zone_cb(self, req):
        # type: (UpdateZoneRequest) -> UpdateZoneResponse
        logger.info('Request to update Zone: {}'.format(req.name))

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        try:
            zone = next(z for z in map_obj.zones if z.name == req.name)  # type: Zone
        except StopIteration:
            return UpdateZoneResponse(
                success=False,
                message='No Zone matching {}'.format(req.name)
            )

        zone.description = req.description
        zone.zone_type = req.zone_type
        zone.polygon = []
        for point in req.polygon.points:
            zone.polygon.append(Point(x=point.x, y=point.y, z=point.z))
        zone.save()

        if req.map_name == self.__map_name:
            self.__publish_zones()
            self.__publish_map()

        return UpdateZoneResponse(
            success=True
        )

    #
    # Internal load functions
    #

    def __load_map(self, map_name):

        logger.info('Loading Map: {}'.format(map_name))

        # Set as the current map
        self.__map_name = str(map_name)

        self.__active_map_pub.publish(String(data=self.__map_name))
        self.__publish_map()
        self.__publish_occupancy_grid()
        self.__publish_zones()

    def __publish_map(self):
        logger.info('Publishing Map')

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map
            self.__map_pub.publish(map_obj.get_msg())

        except Exception as e:
            logger.error('Exception publishing Map: {} - {}'.format(e, traceback.format_exc()))

    def __publish_markers(self):
        logger.info('Publishing Markers')

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map
            self.__markers_pub.publish(map_obj.get_markers_msg())

        except Exception as e:
            logger.error('Exception publishing Markers: {} - {}'.format(e, traceback.format_exc()))

    def __publish_zones(self):
        logger.info('Publishing Zones')

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map
            self.__zones_pub.publish(map_obj.get_zones_msg())

        except Exception as e:
            logger.error('Exception publishing Zones: {} - {}'.format(e, traceback.format_exc()))

    def __publish_occupancy_grid(self):
        logger.info('Publishing Occupancy Grid')

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map
            map_msg = map_obj.map.get_occupancy_grid_msg()

            self.__og_pub.publish(map_msg)
            self.__map_meta_pub.publish(map_msg.info)

        except Exception as e:
            logger.error('Exception publishing OccupancyGrid: {} - {}'.format(e, traceback.format_exc()))
