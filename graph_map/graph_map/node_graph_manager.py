from __future__ import annotations

from typing import Dict, Optional, Tuple, Union, Sequence, Iterable

from graph_map.area import Area
from graph_map.graph_exceptions import DuplicateNodeError, UnkownNodeError, EdgeError
from graph_map.nx_graph_manager import NxGraphManager
from graph_map.area_manager import AreaManager
from graph_map.util import PoseLike

from graph_map.node import Node, Edge

import logging
import networkx as nx

NODE_TYPE = Union[Node, Tuple[Node, Dict]]
EDGE_TYPE = Union[Edge, Tuple[Node, Dict]]
EDGE_ID = Tuple[str, str]
EDGE_ID_NODE = Tuple[Node, Node]

logger = logging.getLogger(__name__)


class NodeGraphManager(NxGraphManager):
    # These keys are reserved for storing member data of the classes when they get converted into JSON
    EDGE_DATA_KEY = '_edge_data'
    NODE_KEY = '_node'

    def __init__(self):
        self.graph: nx.DiGraph = nx.DiGraph()
        self.graph_index_map: Dict[int, str] = dict()

        self.prev_path_graph: Optional[nx.DiGraph] = None
        self.prev_path: Optional[Sequence[Node]] = None
        self.prev_node: Optional[Node] = None
        self.prev_end_node: Optional[Node] = None

        # cached data
        self.__nodes: Optional[Dict[Node, Dict]] = None
        self.__node_ids: Optional[Dict[str, Node]] = None

        self.__area_manager = None

    def _dirty(func):
        def wrapper(self, *args, **kwargs):
            self.__clear_cache()
            return func(self, *args, **kwargs)

        return wrapper

    def clear(self):
        self.graph.clear()

    @_dirty
    def add_node(self, node: Node, attr=None):
        if attr is None:
            attr = dict()

        if node not in self.graph.nodes:
            self.graph.add_node(node, **attr)
            node.set_graph(self.graph)
        else:
            raise DuplicateNodeError('Node {} already exists'.format(node.id))

    @_dirty
    def add_nodes(self, nodes: Iterable[NODE_TYPE]):
        duplicates = list()
        # We aren't using self.graph.add_nodes_from() so that we can check for duplicates
        for node in nodes:
            try:
                if isinstance(node, Node):
                    self.add_node(node)
                else:
                    self.add_node(*node)  # Unpack (node, attr)
            except DuplicateNodeError:
                duplicates.append(node.id)

        if duplicates:
            raise DuplicateNodeError('Duplicate Nodes found: {}'.format(duplicates))

    @_dirty
    def add_edge(self, edge: Edge, attr=None):
        if edge.start not in self.graph.nodes:
            raise UnkownNodeError('Graph does not contain the start node {}'.format(edge.start.id))

        if edge.end not in self.graph.nodes:
            raise UnkownNodeError('Graph does not contain the end node {}'.format(edge.end.id))

        if attr is None:
            attr = dict()

        if self.EDGE_DATA_KEY in attr:
            raise EdgeError('attrs contain reserved keyword ''{}'': {}'.format(self.EDGE_DATA_KEY, attr['_edge_data']))

        self.graph.add_edge(edge.start, edge.end, **attr)

    @_dirty
    def add_edge_by_id(self, start_id, end_id):
        # Needed for external interfaces which will only have the IDs and names
        raise NotImplementedError

    @_dirty
    def add_edges(self, edges: Iterable[EDGE_TYPE]):
        for edge in edges:
            try:
                if isinstance(edge, Edge):
                    self.add_edge(edge)
                else:
                    self.add_edge(*edge)  # Unpack (node, attr)
            except Exception:
                # Placeholder for known exceptions
                raise

    @_dirty
    def remove_edge(self, edge: Edge):
        # NetworkX does raise an exception if the edge does not exist
        self.graph.remove_edge(edge.start, edge.end)

    @_dirty
    def remove_edges(self):
        raise NotImplementedError

    @_dirty
    def remove_node(self):
        raise NotImplementedError

    @_dirty
    def remove_nodes(self):
        raise NotImplementedError

    @_dirty
    def modify_node_attrs(self, node: Node, attr: Dict, clear: bool = False):
        attr_dict = self.graph[node]
        if clear:
            attr_dict.clear()
        attr_dict.update(attr)

    @_dirty
    def modify_edge_attrs(self, edge: Edge, attr: Dict, clear: bool = False):
        attr_dict = self.graph[edge.start][edge.end]
        if clear:
            attr_dict.clear()
        attr_dict.update(attr)

    def shortest_path(self, start: Union[Node, PoseLike], end: Union[Node, PoseLike]) -> Sequence[Node]:
        """
        Finds the shortest path between start and end points.
        @param start: Starting point, if a PoseLike is provided, the closest node will be found
        @param end: Ending point, if a PoseLike is provided, the closest node will be found
        @return: List of Nodes defining the path from start to end
        """
        # Note that we can also compute all_pairs_shortest_path if we want to query multiple solutions
        if not isinstance(start, Node):
            if isinstance(start, PoseLike):
                # Do stuff
                start_node, _ = self.closest_node(start, require_successor=True)
            else:
                raise NotImplementedError('Type {} not implemented'.format(type(start)))
        else:
            start_node = start

        if not isinstance(end, Node):
            if isinstance(start, PoseLike):
                end_node, _ = self.closest_node(end, require_predecessor=True)
            else:
                raise NotImplementedError('Type {} not implemented'.format(type(end)))
        else:
            end_node = end

        if self.prev_path_graph is not None and start_node in self.prev_path_graph and end_node in self.prev_path:
            if end_node == self.prev_end_node:
                shortest_path = self.prev_path

                # Trim the path to the start_node
                try:
                    idx = self.prev_path.index(start_node)
                    self.prev_path = self.prev_path[idx:]
                except ValueError:
                    pass  # no index, so ignore
            else:
                # if the end_node isn't the same then we need to calculate a new path
                logger.info("Previous path subgraph exists, recalculate path")
                shortest_path = nx.shortest_path(
                    self.prev_path_graph, source=start_node, target=end_node, weight='weight')

        else:
            # Get the shortest path from the main graph
            shortest_path = nx.shortest_path(self.graph, source=start_node, target=end_node, weight='weight')
            self.prev_path_graph = self.graph.subgraph(shortest_path).copy()

            # Get all the predecessors of the shortest path and add them into the subgraph of the path
            for node in shortest_path:
                predecessors = self.graph.predecessors(node)
                for pred in predecessors:
                    # Add the predecessor if it's not in the graph
                    # We don't care about the weights of the predecessors (Might need to in future)
                    if pred not in self.prev_path_graph:
                        self.prev_path_graph.add_node(pred)
                        self.prev_path_graph.add_edge(pred, node)

            self.prev_path = shortest_path

        self.prev_node = start_node
        self.prev_end_node = end_node
        return shortest_path

    def closest_node(self, pose: PoseLike,
                     require_predecessor: bool = False, require_successor: bool = False,
                     check_area: bool = True) -> Tuple[Node, float]:
        if check_area:
            closest_area: Area = self.in_area(pose)
            area_nodes = closest_area.nodes
        else:
            area_nodes = list(self.nodes.keys())

        # Only find nodes that have predecessors (ie. nodes that have an edge going towards it)
        if require_predecessor:
            nodes = [node for node in area_nodes if len(list(self.graph.predecessors(node))) >= 1]
        elif require_successor:
            nodes = [node for node in area_nodes if len(list(self.graph.successors(node))) >= 1]
        else:
            nodes = area_nodes

        dists = self.__distances(pose, nodes)
        closest_node = min(dists, key=dists.get)
        return closest_node, dists[closest_node]

    def closest_node_in_list(self, pose: PoseLike, node_list, check_area=True) -> Tuple[Node, float]:
        closest_area: Area = self.in_area(pose)

        # check_area: Only search nodes in the same area as pose
        if check_area:
            in_area_nodes = [node for node in node_list if node.area == closest_area]
        else:
            in_area_nodes = node_list

        if in_area_nodes:
            dists = {node: node.dist(pose) for node in in_area_nodes}
            k = min(dists, key=dists.get)
            return k, dists[k]
        else:
            raise ValueError('No nodes in the area "{}"'.format(closest_area.display_name))

    def in_area(self, pose: PoseLike) -> Area:
        closest_area: Optional[Area] = None
        for area in self.area_manager.get_areas().keys():
            if area.is_in(pose):
                closest_area = area
                break

        if closest_area is None:
            raise ValueError('Could not find Area enclosing pose')

        return closest_area

    @staticmethod
    def __distances(start: PoseLike, node_list: Sequence[Node]) -> Dict[Node, float]:
        return {node: node.dist(start) for node in node_list}

    def subgraph(self, nodes: Sequence[Node]) -> nx.Graph:
        return nx.subgraph(self.graph, nodes)

    def write_graph(self, filepath: str) -> None:
        self._write_graph(filepath, NodeGraphManager.NODE_KEY)

    def to_json(self):
        return self._to_json(NodeGraphManager.NODE_KEY)

    @classmethod
    def read_graph(cls, filepath):
        """
                Reads a networkX graph saved using json_graph.adjacency_data
                :param filepath: Path to graph data
                :return: NodeGraphManager
                """
        gm = cls._read_graph(filepath, NodeGraphManager.NODE_KEY, Node)
        return gm

    @classmethod
    def from_jsons(cls, filepath) -> 'NodeGraphManager':
        """
                Reads a networkX graph saved using json_graph.adjacency_data
                :param filepath: Path to graph data
                :return: NodeGraphManager
                """
        gm = cls._from_jsons(filepath, NodeGraphManager.NODE_KEY, Node)
        return gm

    @property
    def nodes(self):
        """
        Returns a dictionary of nodes and their attributes
        :return:
        """
        if self.__nodes is None:
            self.__nodes = dict(self.graph.nodes)
            self.__node_ids = None

        return self.__nodes

    @property
    def node_ids(self):
        """
        Returns a dictionary of node ids and their nodes
        :return:
        """
        if self.__node_ids is None:
            self.__node_ids = {node.id: node for node in self.nodes.keys()}
        return self.__node_ids

    @property
    def area_manager(self):
        return self.__area_manager

    def get_nodes(self):
        return self.nodes

    def get_graph(self):
        return self.graph

    def set_graph(self, graph: nx.Graph):
        self.graph = graph

    def set_area_manager(self, area_manager: AreaManager):
        self.__area_manager = area_manager
        self.compute_areas()

    def compute_areas(self):
        for node in self.nodes.keys():
            pose = PoseLike(node.x, node.y)
            for area in self.area_manager.get_areas(level=1).keys():
                if area.is_in(pose):
                    node.area = area
                    area.nodes.append(node)
                    break

    def __contains__(self, item):
        if isinstance(item, Node):
            return item in self.nodes
        elif isinstance(item, str):
            # Assume it must be an ID
            return item in self.node_ids
        elif isinstance(item, Edge):
            return item.nx_edge in self.graph.edges
        elif isinstance(item, Sequence):
            if len(item) == 2 or len(item) == 3:
                # (start, end (, attr))
                if isinstance(item[0], str) and isinstance(item[1], str):
                    # Nodes represented with ids
                    return item in self.graph.edges
                elif isinstance(item[0], Node) and isinstance(item[1], Node):
                    # Nodes represented with Nodes
                    return item in self.graph.edges
                else:
                    raise ValueError('Sequence input assumed to be an edge. Start and End nodes must be of type '
                                     'Node or str, got ({}, {})'.format(type(item[0]), type(item[1])))
            else:
                raise ValueError('Sequence input is assumed to be an edge of form '
                                 '\'(start, end (, attr)\'. Item must be length 2 or 3 got {}.'.format(len(item)))
        else:
            return NotImplementedError('Unknown input type: {}'.format(type(item)))

    def __eq__(self, other):
        # This does not test attrs
        if other.graph.edges != self.graph.edges:
            return False

        # This also tests attrs
        snd = dict(self.graph.nodes)
        ond = dict(other.graph.nodes)

        if snd != ond:
            print('Snd: {}\nOnd: {}'.format(snd, ond))

        if other.graph.nodes != self.graph.nodes:
            return False

        return True

    def __clear_cache(self):
        self.__nodes = None
        self.__node_ids = None
        self.prev_path_graph = None
        self.prev_end_node = None
        self.prev_path = None
        self.prev_node = None
