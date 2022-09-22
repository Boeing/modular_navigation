import logging
import traceback
from functools import wraps

import geometry_msgs.msg
#import rospy
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import std_msgs.msg
import typing
from mongoengine import DoesNotExist, ValidationError
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from std_msgs.msg import UInt8MultiArray
from tf2_msgs.msg import TFMessage

from visualization_msgs.msg import MarkerArray as MarkerArrayMsg

from map_manager.srv import AddMap#, AddMapRequest, AddMapResponse
from map_manager.msg import MapInfo as MapInfoMsg
from map_manager.documents import Map
from map_manager.srv import DeleteMap#, DeleteMapRequest, DeleteMapResponse
from map_manager.srv import GetActiveMap#, GetActiveMapRequest, GetActiveMapResponse
from map_manager.srv import GetAreaTree#, GetAreaTreeRequest, GetAreaTreeResponse
from map_manager.srv import GetMapInfo#, GetMapInfoRequest, GetMapInfoResponse
from map_manager.srv import GetNodeGraph#, GetNodeGraphRequest, GetNodeGraphResponse
from map_manager.srv import GetOccupancyGrid#, GetOccupancyGridRequest, GetOccupancyGridResponse
from map_manager.srv import GetZones#, GetZonesRequest, GetZonesResponse
from map_manager.srv import ListMaps#, ListMapsRequest, ListMapsResponse
from map_manager.srv import SetActiveMap#, SetActiveMapRequest, SetActiveMapResponse
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


class RosWrapper(object):
    def __init__(self, node=None):

        #
        # State
        #
        self.__map_name = None

        if node is None:
            logger.info('No node passed to RosWrapper.')
            return

        self.node = node

        """self.__add_map = rospy.Service('~add_map', AddMap, handler=self.__add_map_cb)

        self.__delete_map = rospy.Service('~delete_map', DeleteMap, handler=self.__delete_map_cb)

        self.__get_map_info = rospy.Service('~get_map_info', GetMapInfo, handler=self.__get_map_info_cb)
        self.__get_og = rospy.Service('~get_occupancy_grid', GetOccupancyGrid, handler=self.__get_occupancy_grid_cb)
        self.__get_node_graph = rospy.Service('~get_node_graph', GetNodeGraph, handler=self.__get_node_graph_cb)
        self.__get_area_tree = rospy.Service('~get_area_tree', GetAreaTree, handler=self.__get_area_tree_cb)
        self.__get_zones = rospy.Service('~get_zones', GetZones, handler=self.__get_zones_cb)

        self.__list_maps = rospy.Service('~list_maps', ListMaps, handler=self.__list_maps_cb)

        self.__set_active_map = rospy.Service('~set_active_map', SetActiveMap, handler=self.__set_active_map_cb)
        self.__get_active_map = rospy.Service('~get_active_map', GetActiveMap, handler=self.__get_active_map_cb)
        """
        self.__add_map = self.node.create_service(AddMap, 'add_map', self.__add_map_cb)
        self.__delete_map = self.node.create_service(DeleteMap, 'delete_map', self.__delete_map_cb)

        self.__get_map_info = self.node.create_service(GetMapInfo, 'get_map_info', self.__get_map_info_cb)
        self.__get_og = self.node.create_service(GetOccupancyGrid, 'get_occupancy_grid', self.__get_occupancy_grid_cb)
        self.__get_node_graph = self.node.create_service(GetNodeGraph, 'get_node_graph', self.__get_node_graph_cb)
        self.__get_area_tree = self.node.create_service(GetAreaTree, 'get_area_tree', self.__get_area_tree_cb)
        self.__get_zones = self.node.create_service(GetZones, 'get_zones', self.__get_zones_cb)

        self.__list_maps = self.node.create_service(ListMaps, 'list_maps', self.__list_maps_cb)

        self.__set_active_map = self.node.create_service(SetActiveMap, 'set_active_map' , self.__set_active_map_cb)
        self.__get_active_map = self.node.create_service(GetActiveMap, 'get_active_map', self.__get_active_map_cb)


        # QoS profile, substitutes latch and queue_size args in create_publisher
        # See: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
        self.qos_profile = QoSProfile(depth=100, 
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)


        # Initialise a unit world->map transform
        self.__static_tf_pub = self.node.create_publisher(TFMessage, "/tf_static", self.qos_profile)
        self.__static_tf_pub.publish(
            TFMessage(
                transforms=[
                    geometry_msgs.msg.TransformStamped(
                        header=std_msgs.msg.Header(frame_id='world', stamp=self.node.get_clock().now().to_msg()),
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
        self.__active_map_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__og_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__pbstream_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__zones_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__areas_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__graph_pub = None  # type: typing.Optional[rospy.Publisher]
        self.__init_publishers()

        # Load the most recently modified Map by default
        map_query = Map.objects.order_by('-modified')
        if map_query.count():
            map_obj = map_query.first()
            self.__load_map(map_name=str(map_obj.name))
        else:
            self.__active_map_pub.publish(MapInfoMsg())

        logger.info('Successfully started')

    def __init_publishers(self):
        logger.info('Initialising publishers')

        self.__active_map_pub = self.node.create_publisher(
            MapInfoMsg,
            'active_map',
            qos_profile=self.qos_profile
        )
        self.__og_pub = self.node.create_publisher(
            OccupancyGridMsg,
            'occupancy_grid',
            qos_profile=self.qos_profile
        )
        self.__pbstream_pub = self.node.create_publisher(
            UInt8MultiArray,
            'pbstream',
            qos_profile=self.qos_profile
        )
        self.__zones_pub = self.node.create_publisher(
            MarkerArrayMsg,
            'zones', 
            qos_profile=self.qos_profile
        )
        self.__areas_pub = self.node.create_publisher(
            MarkerArrayMsg,
            'areas',
            qos_profile=self.qos_profile
        )
        self.__graph_pub = self.node.create_publisher(
            MarkerArrayMsg,
            'graph',
            qos_profile=self.qos_profile
        )

    #
    # ADD callbacks
    #

    @exception_wrapper(AddMap.Response)
    def __add_map_cb(self, req):
        # type: (AddMapRequest) -> AddMapResponse
        logger.info('Request to add a new Map: {}'.format(req.map_info.name))

        if len(req.occupancy_grid.data) <= 0:
            return AddMap.Response(
                success=False,
                message='No occupancy grid'
            )

        map_query = Map.objects(name=req.map_info.name)
        if map_query.count():
            map_obj = map_query.first()
            logger.info('Map {} already exists in the database. Overwriting.'.format(req.map_info.name))
            map_obj.delete()
        else:
            map_obj = Map()

        map_obj = Map.from_msg(map_info_msg=req.map_info, zone_msgs=req.zones,
                               occupancy_grid_msg=req.occupancy_grid, pbstream_msg=req.pbstream,
                               node_graph_json=req.node_graph, area_tree_json=req.area_tree)

        map_obj.save()

        self.__load_map(map_name=req.map_info.name)

        return AddMapResponse(
            success=True
        )

    #
    # DELETE callbacks
    #

    @exception_wrapper(DeleteMap.Response)
    def __delete_map_cb(self, req):
        # type: (DeleteMapRequest) -> DeleteMapResponse
        logger.info('Request to delete Map: {}'.format(req.map_name))

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
    def __get_occupancy_grid_cb(self, req):
        # type: (GetOccupancyGridRequest) -> GetOccupancyGridResponse

        # map_name: an empty id will use the currently loaded map
        if not req.map_name:
            if self.__map_name:
                req.map_name = self.__map_name

        map_obj = Map.objects(name=req.map_name).get()

        return GetOccupancyGridResponse(
            grid=map_obj.get_occupancy_grid_msg(self.node),
            success=True
        )

    @exception_wrapper(GetMapInfo.Response)
    def __get_map_info_cb(self, req):
        # type: (GetMapInfoRequest) -> GetMapInfoResponse

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
    def __get_zones_cb(self, req):
        # type: (GetZonesRequest) -> GetZonesResponse

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
    def __get_node_graph_cb(self, req):
        # type: (GetNodeGraphRequest) -> GetNodeGraphResponse

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
    def __get_area_tree_cb(self, req):
        # type: (GetAreaTreeRequest) -> GetAreaTreeResponse

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
    def __list_maps_cb(self, _):
        # type: (ListMapsRequest) -> ListMapsResponse
        return ListMaps.Response(
            active_map=self.__map_name if self.__map_name else '',
            map_infos=[obj.get_map_info_msg() for obj in Map.objects],
            success=True
        )

    #
    # Active map callbacks
    #

    @exception_wrapper(SetActiveMap.Response)
    def __set_active_map_cb(self, req):
        # type: (SetActiveMapRequest) -> SetActiveMapResponse
        logger.info('Request to set active Map: {}'.format(req.map_name))

        Map.objects(name=req.map_name).get()
        self.__load_map(map_name=req.map_name)

        return SetActiveMap.Response(
            success=True
        )

    @exception_wrapper(GetActiveMap.Response)
    def __get_active_map_cb(self, req):
        # type: (GetActiveMapRequest) -> GetActiveMapResponse
        return GetActiveMap.Response(
            active_map=self.__map_name,
            success=True
        )

    #
    # Internal load functions
    #

    def __load_map(self, map_name):

        logger.info('Loading Map: {}'.format(map_name))

        # Set as the current map
        self.__map_name = str(map_name)

        try:
            map_obj = Map.objects(name=self.__map_name).get()  # type: Map

            self.__active_map_pub.publish(map_obj.get_map_info_msg())

            grid = map_obj.get_occupancy_grid_msg(self.node)
            self.__og_pub.publish(grid)

            if map_obj.pbstream:
                self.__pbstream_pub.publish(UInt8MultiArray(data=map_obj.pbstream.read()))
            else:
                self.__pbstream_pub.publish(UInt8MultiArray())

            self.__zones_pub.publish(build_zones_marker_array(self.node, map_obj))
            self.__areas_pub.publish(build_areas_marker_array(self.node, map_obj))
            self.__graph_pub.publish(build_graph_marker_array(self.node, map_obj,
                                                              node_params={'lifetime': rclpy.duration.Duration(seconds=0).to_msg() },
                                                              edge_params={'lifetime': rclpy.duration.Duration(seconds=0).to_msg() }))

        except Exception as e:
            logger.error('Exception publishing data: {} - {}'.format(e, traceback.format_exc()))
