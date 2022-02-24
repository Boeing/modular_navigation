from threading import RLock, Condition
from typing import Sequence, Optional

from graph_map.area import Zone
from graph_map.node import Node
from graph_map.node_graph_manager import NodeGraphManager
from graph_planner.navigation_behavior.behaviors.waypoint_policies.waypoint_parameters import WaypointParameters


class SharedData:
    # Internal state that can't be queried from other sources
    # Need to find a better way to do this

    def __init__(self, gm=None, zones=None, path=None, goal=None, cv=None):
        self.__gm: Optional[NodeGraphManager] = gm
        self.__zones: Optional[Sequence[Zone]] = zones
        self.__path: Optional[Sequence[Node]] = path
        self.__goal: Optional[WaypointParameters] = goal

        self.__autonomy_complete: bool = False
        self.__lock = RLock()

        # Execution condition variable. Can be used (notified to force a plan update)
        self.__cv: Condition = cv

    def update(self, gm=None, zones=None, path=None, goal=None, autonomy_complete=None):
        with self.__lock:
            if gm is not None:
                self.__gm = gm
            if zones is not None:
                self.__zones = zones
            if path is not None:
                self.__path = path
            if goal is not None:
                self.__goal = goal
            if autonomy_complete is not None:
                self.__autonomy_complete = autonomy_complete

    def clear(self):
        with self.__lock:
            self.__gm = None
            self.__zones = None
            self.__path = None
            self.__goal = None

    @property
    def has_gm(self) -> bool:
        with self.__lock:
            return self.__gm is not None

    @property
    def has_zones(self) -> bool:
        with self.__lock:
            return self.__zones is not None

    @property
    def has_path(self) -> bool:
        with self.__lock:
            return self.__path is not None

    @property
    def has_goal(self) -> bool:
        with self.__lock:
            return self.__goal is not None

    @property
    def autonomy_complete(self) -> bool:
        with self.__lock:
            return self.__autonomy_complete

    @property
    def gm(self):
        return self.__gm

    @property
    def zones(self):
        return self.__zones

    @property
    def path(self):
        return self.__path

    @property
    def goal(self):
        return self.__goal

    @property
    def lock(self):
        return self.__lock

    @property
    def cv(self):
        return self.__cv
