import json
import logging
import traceback
from functools import wraps

from graph_map.util import PoseLike
from std_msgs.msg import ColorRGBA

import actionlib
from graph_planner.navigation_behavior.behaviors.control_policies.control_parameters import ControlParameters
from graph_planner.navigation_behavior.behaviors.shared_data import SharedData
import rospy
import tf2_ros
import tf2_geometry_msgs
import typing
import threading

from geometry_msgs.msg import PoseStamped, Point, Pose
from geometry_msgs.msg import Quaternion as QuaternionMsg
from std_msgs.msg import Header

from graph_planner.srv import LoadFloorplan, LoadFloorplanResponse, LoadFloorplanRequest, \
    GraphPlan, GraphPlanResponse, GraphPlanRequest
from map_manager.msg import MapInfo as MapInfoMsg

from graph_map.node_graph_manager import NodeGraphManager
from graph_map.area_manager import AreaManager
from graph_map.area import Zone
from map_manager.dxf.dxf_loader import DxfLoader
from map_manager.srv import GetNodeGraph, GetNodeGraphRequest, GetNodeGraphResponse
from map_manager.srv import GetAreaTree, GetAreaTreeRequest, GetAreaTreeResponse
from map_manager.srv import GetZones, GetZonesRequest, GetZonesResponse
from map_manager.visualise import build_graph_marker_array

from graph_planner.navigation_behavior.behaviors.waypoint_policies.waypoint_parameters import PoseSampling, \
    PoseTolerance, WaypointParameters
from graph_planner.navigation_behavior.behaviors.policy.status import Status
from graph_planner.navigation_behavior.behavior_selector import BehaviourSelector
from graph_planner.navigation_behavior.behaviors.node_follow import NodeFollow
from graph_planner.navigation_behavior.behaviors.edge_intervention import EdgeIntervention

from cartographer_ros_msgs.msg import SystemState

from autonomy.msg import DriveAction, DriveGoal, DriveFeedback, DriveResult

from visualization_msgs.msg import MarkerArray

logger = logging.getLogger(__name__)


def exception_wrapper(return_cls):
    def wrapper(func):
        @wraps(func)
        def func_wrapper(self, msg, *args, **kwargs):
            try:
                return func(self, msg, *args, **kwargs)
            except Exception as e:
                msg = 'exception_wrapper: exception: {}: {}'.format(str(e), traceback.format_exc())
                logger.error(msg)
                return return_cls(success=False, message=msg)

        return func_wrapper

    return wrapper


# Interface:
#    - load stuff from active_map
#      - needs to update when active_map updates
#      - Need to wait for an active_map and not fail
#    - Given a target goal, provide a A* goal
# Finding shortest path subgraph is enclosed and passed directly to the behavioral layer
#    This is so the behavioral layer works on the same in memory graph
class RosWrapper(object):
    def __init__(self, planning_frame='map'):
        self.__update_period = rospy.get_param('~/update_period', default=2.0)
        self.__planning_frame = planning_frame

        self.__cv = threading.Condition()
        self.shared_data = SharedData(cv=self.__cv)
        self.behavior_selector = BehaviourSelector(shared_data=self.shared_data)
        self.behavior_selector.register_behavior(EdgeIntervention())
        self.behavior_selector.register_behavior(NodeFollow())
        self.__current_status = Status.UNKNOWN

        self.__tf_buffer = tf2_ros.Buffer()
        self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer)

        self.__localised = False
        self.__mapper_status_subscriber = rospy.Subscriber(
            name='/mapper/state',
            data_class=SystemState,
            callback=self.__mapper_cb
        )
        # self.__odom_subscriber = rospy.Subscriber(
        #     name='odom',
        #     data_class=Odometry,
        #     callback=self.__odom_callback
        # )

        # Subscribe to the active map
        self.__active_map_subscriber = rospy.Subscriber(
            name='/map_manager/active_map',
            data_class=MapInfoMsg,
            callback=self.active_map_cb
        )

        # Services
        self.__load_floorplan = rospy.Service('~load_floorplan', LoadFloorplan, handler=self.__load_floorplan_cb)
        self.__plan = rospy.Service('~shortest_path', GraphPlan, handler=self.__plan_cb)

        # Goal Action Server (receives goals from others)
        self.__goal: typing.Optional[DriveGoal] = None
        self.__action_server = actionlib.SimpleActionServer(rospy.get_name(), DriveAction, auto_start=False)
        self.__action_server.register_goal_callback(self.__new_goal_cb)
        self.__action_server.register_preempt_callback(self.__cancel_cb)

        # Autonomy Action Client (sends goals to autonomy)
        self.__action_client = actionlib.SimpleActionClient('autonomy', DriveAction)
        logger.info('Waiting for autonomy Action Server...')
        self.__action_client.wait_for_server(timeout=rospy.Duration(30))

        self.__subgraph_pub = rospy.Publisher('~path_subgraph', data_class=MarkerArray, latch=True, queue_size=2)

        # Wait for a map
        logger.info("Waiting for a map to be loaded...")
        while not self.behavior_selector.shared_data.has_gm and not rospy.is_shutdown():
            rospy.sleep(1)

        # Only start server once connected to autonomy and a map has been loaded
        self.__action_server.start()
        logger.info('Started graph planner Action Server')

        # Create a thread to execute the planner
        run_thread = threading.Thread(target=self.__run_planner)

        run_thread.start()
        logger.info('Started Planning thread')
        rospy.spin()
        with self.__cv:
            self.__cv.notify_all()
        run_thread.join()
        logger.info('Planning thread ended')

    def __run_planner(self):
        with self.__cv:
            while not rospy.is_shutdown():
                if not self.__localised:
                    logger.info('Robot not localised. Waiting...')
                    self.__cv.wait(timeout=2)
                    continue

                # Get shortest path
                if self.__goal is not None:
                    logger.info('Running graph planner')

                    start_pose = PoseLike.from_msg(self.__get_robot_pose().pose)
                    end_pose = PoseLike.from_msg(self.__goal.target_pose.pose)

                    with self.behavior_selector.shared_data.lock:
                        shortest = self.behavior_selector.shared_data.gm.shortest_path(start=start_pose, end=end_pose)
                        subgraph = self.behavior_selector.shared_data.gm.prev_path_graph

                        self.__subgraph_pub.publish(build_graph_marker_array(
                            subgraph,
                            node_params={'color': ColorRGBA(0.1, 0.1, 0.6, 0.9), 'radius': 0.5},
                            edge_params={'color': ColorRGBA(0.2, 0.8, 0.2, 0.7), 'diameter': 0.15}))

                    if len(shortest) == 0:
                        logger.error("Unable to find a path between {} and {}. Check your traversal graph"
                                     .format(start_pose, end_pose))
                        self.__cv.wait(timeout=self.__update_period)
                        continue

                    self.behavior_selector.update_graph_plan(shortest)

                    outcome = self.behavior_selector.update(start_pose)
                    self.__current_status = outcome.status
                    logger.info('Status: {}'.format(self.__current_status))

                    if outcome.waypoint is not None and \
                            (self.__current_status == Status.RUNNING or self.__current_status == Status.SUCCESS):
                        waypoint = outcome.waypoint.parameters
                        waypoint_pose = PoseStamped(
                            header=Header(
                                stamp=rospy.Time.now(),
                                frame_id=self.__planning_frame
                            ),
                            pose=waypoint.pose.to_msg()
                        )

                        if outcome.control is None:
                            control = ControlParameters()
                        else:
                            control = outcome.control.parameters

                        self._goal_cmd = DriveGoal(
                            target_pose=waypoint_pose,
                            std_x=waypoint.sampling.std_x,
                            std_y=waypoint.sampling.std_y,
                            std_w=waypoint.sampling.std_w,
                            max_samples=waypoint.sampling.max_samples,
                            xy_goal_tolerance=waypoint.tolerance.xy_goal_tolerance,
                            yaw_goal_tolerance=waypoint.tolerance.yaw_goal_tolerance,

                            # Control parameters (passthrough)
                            max_velocity_x=control.max_velocity.x,
                            max_velocity_y=control.max_velocity.y,
                            max_velocity_w=control.max_velocity.w,
                            avoid_distance=control.avoid_distance,
                            backwards_mult=control.multipliers.backwards_mult,
                            strafe_mult=control.multipliers.strafe_mult,
                            rotation_mult=control.multipliers.rotation_mult,
                        )

                        logger.info('Sending new waypoint to Autonomy: ({}, {})'.
                                    format(waypoint.pose.x, waypoint.pose.y))

                        self.__action_client.send_goal(self._goal_cmd, feedback_cb=self.__feedback_cb,
                                                       done_cb=self.__done_cb)
                        self.behavior_selector.shared_data.update(autonomy_complete=False)

                    self.__cv.wait(timeout=self.__update_period)
                else:
                    self.__cv.wait()

    #
    # Map methods
    #
    def active_map_cb(self, map_info: MapInfoMsg):
        logger.info('Active map changed: {}'.format(map_info.name))

        # Get area tree
        logger.info("Loading Area Tree...")
        get_area_tree_srv = rospy.ServiceProxy(
            name='map_manager/get_area_tree',
            service_class=GetAreaTree
        )
        get_area_tree_srv.wait_for_service(timeout=10)
        response = get_area_tree_srv.call(GetAreaTreeRequest(map_name=map_info.name))
        assert isinstance(response, GetAreaTreeResponse)
        if not response.success:
            raise Exception('Failed to get area tree: {}'.format(response.status_message))

        am = AreaManager.from_jsons(response.area_tree)

        # Get graph map
        logger.info("Loading Graph Map...")
        get_node_graph_srv = rospy.ServiceProxy(
            name='map_manager/get_node_graph',
            service_class=GetNodeGraph
        )
        get_node_graph_srv.wait_for_service(timeout=10)
        response = get_node_graph_srv.call(GetNodeGraphRequest(map_name=map_info.name))
        assert isinstance(response, GetNodeGraphResponse)
        if not response.success:
            raise Exception('Failed to get node graph: {}'.format(response.status_message))

        gm = NodeGraphManager.from_jsons(response.node_graph)

        gm.set_area_manager(am)

        # Get zones
        logger.info("Loading Zones...")
        get_zones_srv = rospy.ServiceProxy(
            name='map_manager/get_zones',
            service_class=GetZones
        )
        get_zones_srv.wait_for_service(timeout=10)
        response = get_zones_srv.call(GetZonesRequest(map_name=map_info.name))
        assert isinstance(response, GetZonesResponse)
        if not response.success:
            raise Exception('Failed to get node graph: {}'.format(response.status_message))

        zones = [Zone.from_msg(zone_msg) for zone_msg in response.zones]

        self.behavior_selector.update_map_data(gm=gm, zones=zones)

        # Invalidate any current plans
        self.behavior_selector.clear_states()
        self.__action_client.cancel_all_goals()
        if self.__action_server is not None:
            if self.__action_server.is_active():
                self.__action_server.current_goal.set_aborted(text='Goal aborted due to map update')

        logger.info("Map loaded!")

    def load_dxf(self, fpath):
        dxf_loader = DxfLoader(fpath)
        gm = dxf_loader.parse_dxf()
        zones = [dxf_loader.zones.values()]
        self.behavior_selector.update_map_data(gm=gm, zones=zones)

    @exception_wrapper(LoadFloorplanResponse)
    def __load_floorplan_cb(self, req: LoadFloorplanRequest) -> LoadFloorplanResponse:
        logger.info('Importing floorplan from DXF: {}'.format(req.floorplan_name))

        resp = LoadFloorplanResponse()

        with self.behavior_selector.shared_data.lock:
            try:
                self.load_dxf(fpath=req.floorplan_name)
                msg = 'Graph has been loaded: {}'.format(self.behavior_selector.shared_data.gm)
                logger.info(msg)
                resp.floorplan_json = json.dumps(self.behavior_selector.shared_data.gm.graph.__dict__)  # Placeholder
                resp.success = True
            except Exception as e:
                resp.success = False
                resp.message = str(e)

        return resp

    def __mapper_cb(self, msg: SystemState):
        if msg.localisation_status == SystemState.LOCALISED or msg.localisation_status == SystemState.PAUSED:
            if not self.__localised:
                self.__localised = True
                logger.info('Robot localised!')
        else:
            if self.__localised:
                self.__localised = False
                logger.info('Robot no longer localised!')

    def __get_robot_pose(self):
        try:
            pose = self.__tf_buffer.transform(
                PoseStamped(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id='base_link'
                    ),
                    pose=Pose(
                        position=Point(0, 0, 0),
                        orientation=QuaternionMsg(0, 0, 0, 1)
                    )),
                target_frame=self.__planning_frame,
                timeout=rospy.Duration(1.0)
            )
        except Exception as e:
            logger.error(e)
            return None

        return pose

    #
    # Action server callbacks
    #
    def __new_goal_cb(self):
        new_goal: DriveGoal = self.__action_server.next_goal.get_goal()
        assert(isinstance(new_goal, DriveGoal))
        logger.info('Received new goal, x: {}, y: {}'.format(new_goal.target_pose.pose.position.x,
                                                             new_goal.target_pose.pose.position.y))

        self.__action_client.cancel_all_goals()

        if not self.behavior_selector.shared_data.has_gm:
            logger.info('No map loaded, rejecting goal')
            self.__action_server.next_goal.set_rejected()
            return

        # Goals must be in planning frame (default is 'map')
        if new_goal.target_pose.header.frame_id != self.__planning_frame:
            logger.info('Goal is not in frame: {}. Transforming.'.format(self.__planning_frame))
            try:
                tf = self.__tf_buffer.lookup_transform(
                    target_frame=self.__planning_frame,
                    source_frame=new_goal.target_pose.header.frame_id,
                    time=rospy.Time.now(),
                    timeout=rospy.Duration(10))
            except Exception as e:
                logger.error(e)
                self.__action_server.next_goal.set_rejected()
                return

            new_goal.target_pose = tf2_geometry_msgs.do_transform_pose(new_goal.target_pose, tf)

        goal_waypoint_parameters = WaypointParameters(
            pose=PoseLike.from_msg(new_goal.target_pose.pose),
            sampling=PoseSampling(
                std_x=new_goal.std_x,
                std_y=new_goal.std_y,
                std_w=new_goal.std_w,
                max_samples=new_goal.max_samples
            ),
            tolerance=PoseTolerance(
                xy_goal_tolerance=new_goal.xy_goal_tolerance,
                yaw_goal_tolerance=new_goal.yaw_goal_tolerance
            )
        )

        self.__action_server.accept_new_goal()

        with self.__cv:
            self.__goal = new_goal
            self.behavior_selector.clear_states()
            self.behavior_selector.update_goal(goal_waypoint_parameters)
            self.__cv.notify_all()

    def __cancel_cb(self):
        self.__action_client.cancel_all_goals()

        with self.__cv:
            self.__goal = None
            self.__cv.notify_all()

    def __feedback_cb(self, msg: DriveFeedback):
        # Forward feedback from autonomy to user
        self.__action_server.publish_feedback(msg)

    def __done_cb(self, status, result: DriveResult):
        with self.__cv:
            if self.__goal is not None:
                # Check if all behaviours are finished
                completed = list()
                statuses = list()
                for behavior in self.behavior_selector.behaviors:
                    statuses.append(behavior.last_outcome.status)
                    completed.append(behavior.last_outcome.status == Status.SUCCESS or
                                     behavior.last_outcome.status == Status.NOT_RUNNING or
                                     behavior.last_outcome.status == Status.UNKNOWN)

                logger.info('Autonomy goal reached: {}'.format(statuses))
                if all(completed):
                    self.__action_server.set_succeeded(result=result)
                    self.__goal = None
                    logger.info('Goal complete!')

            self.behavior_selector.shared_data.update(autonomy_complete=True)
            self.__cv.notify_all()

    #
    # ROS interface
    #
    @exception_wrapper(GraphPlanResponse)
    def __plan_cb(self, req: GraphPlanRequest) -> GraphPlanResponse:
        resp = GraphPlanResponse()
        if not self.behavior_selector.shared_data.has_gm:
            msg = 'No active graph manager found, /map_manager/active_map must be set, or DXF must be loaded'
            logger.warning(msg)
            resp.success = False
            resp.message = msg
        else:
            logger.info('Running plan')
            resp.success = True
            resp.floorplan_json = ''

        return resp
