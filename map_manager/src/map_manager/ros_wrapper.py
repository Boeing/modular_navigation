import logging
import traceback
from functools import wraps

import geometry_msgs.msg
import numpy
import rospy
import std_msgs.msg
import typing
from PIL import Image
from mongoengine import DoesNotExist, ValidationError
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from hd_map.msg import Map as MapMsg
from hd_map.msg import Marker as MarkerMsg
from hd_map.msg import Zone as ZoneMsg
from map_manager.documents import Map, Zone, Marker, OccupancyGrid, Pose, Point, Quaternion
from map_manager.srv import AddMap, AddMapRequest, AddMapResponse
from map_manager.srv import DeleteMap, DeleteMapRequest, DeleteMapResponse
from map_manager.srv import GetMap, GetMapRequest, GetMapResponse
from map_manager.srv import GetOccupancyGrid, GetOccupancyGridRequest, GetOccupancyGridResponse
from map_manager.srv import ListMaps, ListMapsRequest, ListMapsResponse
from map_manager.srv import SetActiveMap, SetActiveMapRequest, SetActiveMapResponse
from map_manager.srv import UpdateMap, UpdateMapResponse, UpdateMapRequest

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

        self.__delete_map = rospy.Service('~delete_map', DeleteMap, handler=self.__delete_map_cb)

        self.__get_map = rospy.Service('~get_map', GetMap, handler=self.__get_map_cb)
        self.__get_og = rospy.Service('~get_occupancy_grid', GetOccupancyGrid, handler=self.__get_occupancy_grid_cb)

        self.__list_map = rospy.Service('~list_maps', ListMaps, handler=self.__list_maps_cb)

        self.__list_zone = rospy.Service('~set_active_map', SetActiveMap, handler=self.__set_active_map_cb)

        self.__update_map = rospy.Service('~update_map', UpdateMap, handler=self.__update_map_cb)

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
        logger.info('Request to add a new Map: {}'.format(req.map.name))

        if len(req.grid.data) <= 0:
            return AddMapResponse(
                success=False,
                message='No map data'
            )

        assert isinstance(req.grid, OccupancyGridMsg)
        og_map = OccupancyGrid()
        og_map.width = req.grid.info.width
        og_map.height = req.grid.info.height
        og_map.resolution = req.grid.info.resolution
        og_map.origin = Pose(
            position=Point(
                x=req.grid.info.origin.position.x,
                y=req.grid.info.origin.position.y,
                z=req.grid.info.origin.position.z),
            quaternion=Quaternion(
                x=req.grid.info.origin.orientation.x,
                y=req.grid.info.origin.orientation.y,
                z=req.grid.info.origin.orientation.z,
                w=req.grid.info.origin.orientation.w
            )
        )
        im = Image.fromarray(numpy.asarray(req.grid.data, dtype=numpy.uint8)
                             .reshape(req.grid.info.height, req.grid.info.width))
        og_map.image.new_file()
        im.save(fp=og_map.image, format='PPM')
        og_map.image.close()

        obj = Map(
            name=req.map.name,
            description=req.map.description,
            map=og_map
        )

        for marker in req.map.markers:
            assert isinstance(marker, MarkerMsg)
            m_obj = Marker(
                name=marker.name,
                description=marker.description,
                marker_type=marker.marker_type,
                pose=Pose(
                    position=Point(
                        x=marker.pose.position.x,
                        y=marker.pose.position.y,
                        z=marker.pose.position.z
                    ),
                    quaternion=Quaternion(
                        w=marker.pose.orientation.w,
                        x=marker.pose.orientation.x,
                        y=marker.pose.orientation.y,
                        z=marker.pose.orientation.z
                    )
                )
            )
            obj.markers.append(m_obj)

        for zone in req.map.zones:
            assert isinstance(zone, ZoneMsg)
            obj = Zone(
                name=zone.name,
                description=zone.description,
                zone_type=zone.zone_type,
                polygon=[Point(x=point.x, y=point.y, z=point.z) for point in zone.polygon.points]
            )

        obj.save()

        self.__load_map(map_name=req.map.name)

        return AddMapResponse(
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

        if req.map.data:
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
            map_obj.map = og_map

        for marker_msg in req.map.markers:
            assert isinstance(marker_msg, MarkerMsg)
            try:
                marker = next(m for m in map_obj.markers if m.name == marker_msg.name)  # type: Marker
            except StopIteration:
                return UpdateMapResponse(
                    success=False,
                    message='No Marker matching {}'.format(marker_msg.name)
                )

            marker.description = marker_msg.description
            marker.marker_type = marker_msg.marker_type
            marker.pose = Pose(
                position=Point(
                    x=marker_msg.pose.position.x,
                    y=marker_msg.pose.position.y,
                    z=marker_msg.pose.position.z
                ),
                quaternion=Quaternion(
                    w=marker_msg.pose.orientation.w,
                    x=marker_msg.pose.orientation.x,
                    y=marker_msg.pose.orientation.y,
                    z=marker_msg.pose.orientation.z
                )
            )
            marker.save()

        for zone_msg in req.map.zones:
            assert isinstance(zone_msg, ZoneMsg)
            try:
                zone = next(z for z in map_obj.zones if z.name == zone_msg.name)  # type: Zone
            except StopIteration:
                return UpdateMapResponse(
                    success=False,
                    message='No Zone matching {}'.format(zone_msg.name)
                )

            zone.description = zone_msg.description
            zone.zone_type = zone_msg.zone_type
            zone.polygon = []
            for point in zone_msg.polygon.points:
                zone.polygon.append(Point(x=point.x, y=point.y, z=point.z))
            zone.save()

        if req.map.description:
            map_obj.description = req.map.description

        map_obj.save()

        if req.map_name == self.__map_name:
            self.__publish_map()

        return UpdateMapResponse(
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

    def __publish_map(self):
        logger.info('Publishing Map')

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map
            self.__map_pub.publish(map_obj.get_msg())

        except Exception as e:
            logger.error('Exception publishing Map: {} - {}'.format(e, traceback.format_exc()))

    def __publish_occupancy_grid(self):
        logger.info('Publishing Occupancy Grid')

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map
            map_msg = map_obj.map.get_occupancy_grid_msg()

            self.__og_pub.publish(map_msg)

        except Exception as e:
            logger.error('Exception publishing OccupancyGrid: {} - {}'.format(e, traceback.format_exc()))
