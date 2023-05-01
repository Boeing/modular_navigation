import logging
import traceback
from functools import wraps

import mongoengine
import pymongo

import geometry_msgs.msg
# import rospy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
import std_msgs.msg
import typing
from mongoengine import DoesNotExist, ValidationError
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from std_msgs.msg import UInt8MultiArray
from tf2_msgs.msg import TFMessage

from map_manager.config import DATABASE_NAME

from visualization_msgs.msg import MarkerArray as MarkerArrayMsg

from map_manager.srv import AddMap  # , AddMapRequest, AddMapResponse
from map_manager.msg import MapInfo as MapInfoMsg
from map_manager.documents import Map
from map_manager.srv import DeleteMap  # , DeleteMapRequest, DeleteMapResponse
from map_manager.srv import GetActiveMap  # , GetActiveMapRequest, GetActiveMapResponse
from map_manager.srv import GetAreaTree  # , GetAreaTreeRequest, GetAreaTreeResponse
from map_manager.srv import GetMapInfo  # , GetMapInfoRequest, GetMapInfoResponse
from map_manager.srv import GetNodeGraph  # , GetNodeGraphRequest, GetNodeGraphResponse
from map_manager.srv import GetOccupancyGrid  # , GetOccupancyGridRequest, GetOccupancyGridResponse
from map_manager.srv import GetZones  # , GetZonesRequest, GetZonesResponse
from map_manager.srv import ListMaps  # , ListMapsRequest, ListMapsResponse
from map_manager.srv import SetActiveMap  # , SetActiveMapRequest, SetActiveMapResponse
from map_manager.visualise import build_zones_marker_array, build_areas_marker_array, build_graph_marker_array

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


class RosWrapper(Node):
    def __init__(self, node=None):

        #
        # State
        #
        super().__init__('map_manager')


        self.__map_name = None

        # Parameters are now handled by the node, no more param server
        self.mongo_hostname = self.declare_parameter('~mongo_hostname', 'localhost').value
        self.mongo_port = self.declare_parameter('~mongo_port', 27017).value

        self.logger = self.get_logger()

        #
        # Connect to the db
        #
        self.logger.info('Connecting to {}:{}'.format(self.mongo_hostname, self.mongo_port))
        try:
            database = mongoengine.connect(
                db=DATABASE_NAME,
                host=self.mongo_hostname,
                port=self.mongo_port,
                serverSelectionTimeoutMS=30)
        except mongoengine.ConnectionFailure as e:
            self.logger.error("Failed to connect to Mongodb", exc_info=e)
            raise e

        #
        # Force check to make sure Mongo is alive
        #
        try:
            database.server_info()
        except pymongo.errors.ServerSelectionTimeoutError as e:
            self.logger.error("Mongodb is offline", exc_info=e)
            raise e

        self.__add_map = self.create_service(AddMap, self.get_name() + '/add_map', self.__add_map_cb)
        self.__delete_map = self.create_service(DeleteMap, self.get_name() + '/delete_map', self.__delete_map_cb)

        self.__get_map_info = self.create_service(GetMapInfo, self.get_name() + '/get_map_info', self.__get_map_info_cb)
        self.__get_og = self.create_service(GetOccupancyGrid, self.get_name() + '/get_occupancy_grid', self.__get_occupancy_grid_cb)
        self.__get_node_graph = self.create_service(GetNodeGraph, self.get_name() + '/get_node_graph', self.__get_node_graph_cb)
        self.__get_area_tree = self.create_service(GetAreaTree, self.get_name() + '/get_area_tree', self.__get_area_tree_cb)
        self.__get_zones = self.create_service(GetZones, self.get_name() + '/get_zones', self.__get_zones_cb)

        self.__list_maps = self.create_service(ListMaps, self.get_name() + '/list_maps', self.__list_maps_cb)

        self.__set_active_map = self.create_service(SetActiveMap, self.get_name() + '/set_active_map', self.__set_active_map_cb)
        self.__get_active_map = self.create_service(GetActiveMap, self.get_name() + '/get_active_map', self.__get_active_map_cb)

        # QoS profile, substitutes latch and queue_size args in create_publisher
        # See: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
        # Using geometry2 latching params:
        #https://github.com/ros2/geometry2/blob/6788c1b78fe35e1a459a9d8f184afeb79792bd71/tf2_ros/src/tf2_ros/static_transform_broadcaster.py#L55
        self.qos_profile = QoSProfile(depth=100,
                                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                      history=HistoryPolicy.KEEP_LAST,
                                      )
        # Initialise a unit world->map transform
        self.__static_tf_pub = self.create_publisher(TFMessage, "/tf_static", self.qos_profile)
        self.__static_tf_pub.publish(
            TFMessage(
                transforms=[
                    geometry_msgs.msg.TransformStamped(
                        header=std_msgs.msg.Header(frame_id='world', stamp=self.get_clock().now().to_msg()),
                        child_frame_id='map',
                        transform=geometry_msgs.msg.Transform(
                            translation=geometry_msgs.msg.Vector3(x=0., y=0., z=0.),
                            rotation=geometry_msgs.msg.Quaternion(x=0., y=0., z=0., w=1.)
                        )
                    )
                ]
            )
        )

        #
        # Publishers
        #
        self.__active_map_pub = None
        self.__og_pub: typing.Optional[rclpy.Publisher] = None
        self.__pbstream_pub = None
        self.__zones_pub = None
        self.__areas_pub = None
        self.__graph_pub = None
        self.__init_publishers()

        # Load the most recently modified Map by default
        map_query = Map.objects.order_by('-modified')
        if map_query.count():
            map_obj = map_query.first()
            self.__load_map(map_name=str(map_obj.name))  # __load_map loads map even without rcl.spin()
        else:
            self.__active_map_pub.publish(MapInfoMsg())

        self.logger.info('Successfully started')

        # Publish every topic with a timer callback, TODO just in case latching fails
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

    def __init_publishers(self):
        self.logger.info('Initialising publishers')

        self.__active_map_pub = self.create_publisher(
            MapInfoMsg,
            self.get_name() + '/active_map',
            qos_profile=self.qos_profile
        )
        self.__og_pub = self.create_publisher(
            OccupancyGridMsg,
            self.get_name() + '/occupancy_grid',
            qos_profile=self.qos_profile
        )
        self.__pbstream_pub = self.create_publisher(
            UInt8MultiArray,
            self.get_name() + '/pbstream',
            qos_profile=self.qos_profile
        )
        self.__zones_pub = self.create_publisher(
            MarkerArrayMsg,
            self.get_name() + '/zones',
            qos_profile=self.qos_profile
        )
        self.__areas_pub = self.create_publisher(
            MarkerArrayMsg,
            self.get_name() + '/areas',
            qos_profile=self.qos_profile
        )
        self.__graph_pub = self.create_publisher(
            MarkerArrayMsg,
            self.get_name() + '/graph',
            qos_profile=self.qos_profile
        )

    def timer_callback(self):
        """Callback to publish map_manager topics given a timer
        """

        map_obj = Map.objects(name=self.__map_name).get()  # type: Map

        self.__active_map_pub.publish(map_obj.get_map_info_msg())

        grid = map_obj.get_occupancy_grid_msg(self)
        self.__og_pub.publish(grid)

        if map_obj.pbstream:
            self.__pbstream_pub.publish(UInt8MultiArray(data=map_obj.pbstream.read()))
        else:
            self.__pbstream_pub.publish(UInt8MultiArray())

        self.__zones_pub.publish(build_zones_marker_array(self, map_obj))
        self.__areas_pub.publish(build_areas_marker_array(self, map_obj))
        self.__graph_pub.publish(
            build_graph_marker_array(
                self, map_obj, node_params={'lifetime': rclpy.duration.Duration(seconds=0).to_msg()},
                edge_params={'lifetime': rclpy.duration.Duration(seconds=0).to_msg()}))

    #
    # ADD callbacks
    #

    @exception_wrapper(AddMap.Response)
    def __add_map_cb(self, req: AddMap.Request, res) -> AddMap.Response:
        self.logger.info('Request to add a new Map: {}'.format(req.map_info.name))

        if len(req.occupancy_grid.data) <= 0:
            return AddMap.Response(
                success=False,
                message='No occupancy grid'
            )

        map_query = Map.objects(name=req.map_info.name)
        if map_query.count():
            map_obj = map_query.first()
            self.logger.info('Map {} already exists in the database. Overwriting.'.format(req.map_info.name))
            map_obj.delete()
        else:
            map_obj = Map()

        map_obj = Map.from_msg(map_info_msg=req.map_info, zone_msgs=req.zones,
                               occupancy_grid_msg=req.occupancy_grid, pbstream_msg=req.pbstream,
                               node_graph_json=req.node_graph, area_tree_json=req.area_tree)

        map_obj.save()

        self.__load_map(map_name=req.map_info.name)

        return AddMap.Response(
            success=True
        )

    #
    # DELETE callbacks
    #

    @exception_wrapper(DeleteMap.Response)
    def __delete_map_cb(self, req: DeleteMap.Request, res) -> DeleteMap.Response:
        self.logger.info('Request to delete Map: {}'.format(req.map_name))

        obj = Map.objects(name=req.map_name).get()
        obj.delete()

        if req.map_name == self.__map_name:
            self.__init_publishers()
            self.__map_name = None

        return DeleteMap.Response(
            success=True
        )

    #
    # GET callbacks
    #

    @exception_wrapper(GetOccupancyGrid.Response)
    def __get_occupancy_grid_cb(self, req: GetOccupancyGrid.Request, res) -> GetOccupancyGrid.Response:
        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return GetOccupancyGrid.Response(
            grid=map_obj.get_occupancy_grid_msg(self),
            success=True
        )

    @exception_wrapper(GetMapInfo.Response)
    def __get_map_info_cb(self, req: GetMapInfo.Request, res) -> GetMapInfo.Response:
        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return GetMapInfo.Response(
            map_info=map_obj.get_map_info_msg(),
            success=True
        )

    @exception_wrapper(GetZones.Response)
    def __get_zones_cb(self, req: GetZones.Request, res) -> GetZones.Response:
        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj: Map = Map.objects(name=req.map_name).get()

        return GetZones.Response(
            zones=[zone.get_msg() for zone in map_obj.zones],
            success=True
        )

    @exception_wrapper(GetNodeGraph.Response)
    def __get_node_graph_cb(self, req: GetNodeGraph.Request, res) -> GetNodeGraph.Response:
        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj: Map = Map.objects(name=req.map_name).get()

        return GetNodeGraph.Response(
            node_graph=map_obj.node_graph,
            success=True
        )

    @exception_wrapper(GetAreaTree.Response)
    def __get_area_tree_cb(self, req: GetAreaTree.Request, res) -> GetAreaTree.Response:
        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj: Map = Map.objects(name=req.map_name).get()

        return GetAreaTree.Response(
            area_tree=map_obj.area_tree,
            success=True
        )

    #
    # LIST callbacks
    #

    @exception_wrapper(ListMaps.Response)
    def __list_maps_cb(self, _: ListMaps.Request, res) -> ListMaps.Response:
        return ListMaps.Response(
            active_map=self.__map_name if self.__map_name else '',
            map_infos=[obj.get_map_info_msg() for obj in Map.objects],
            success=True
        )

    #
    # Active map callbacks
    #

    @exception_wrapper(SetActiveMap.Response)
    def __set_active_map_cb(self, req: SetActiveMap.Request, res) -> SetActiveMap.Response:
        self.logger.info('Request to set active Map: {}'.format(req.map_name))

        Map.objects(name=req.map_name).get()
        self.__load_map(map_name=req.map_name)

        return SetActiveMap.Response(
            success=True
        )

    @exception_wrapper(GetActiveMap.Response)
    def __get_active_map_cb(self, req: GetActiveMap.Request, res) -> GetActiveMap.Response:
        return GetActiveMap.Response(
            map_name=self.__map_name,
            success=True
        )

    #
    # Internal load functions
    #

    def __load_map(self, map_name):

        self.logger.info('Loading Map: {}'.format(map_name))

        # Set as the current map
        self.__map_name = str(map_name)

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map

            self.__active_map_pub.publish(map_obj.get_map_info_msg())

            grid = map_obj.get_occupancy_grid_msg(self)
            self.__og_pub.publish(grid)

            if map_obj.pbstream:
                self.__pbstream_pub.publish(UInt8MultiArray(data=map_obj.pbstream.read()))
            else:
                self.__pbstream_pub.publish(UInt8MultiArray())

            self.__zones_pub.publish(build_zones_marker_array(self, map_obj))
            self.__areas_pub.publish(build_areas_marker_array(self, map_obj))
            self.__graph_pub.publish(
                build_graph_marker_array(
                    self, map_obj, node_params={'lifetime': rclpy.duration.Duration(seconds=0).to_msg()},
                    edge_params={'lifetime': rclpy.duration.Duration(seconds=0).to_msg()}))
            self.logger.info('Map: {} is Loaded'.format(map_name))
        except Exception as e:
            self.logger.error('Exception publishing data: {} - {}'.format(e, traceback.format_exc()))
