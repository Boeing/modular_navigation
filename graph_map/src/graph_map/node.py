from __future__ import annotations  # postpone type evaluation

from typing import Dict, Optional, Final, Union

import numpy as np

from graph_map.util import to_simple, dunder_name, Versioned, PoseLike
from graph_map.area import Area
from graph_map.graph_exceptions import ParentExistsNodeError, DuplicateChildNodeError, ParallelEdgeError
from graph_map.util import SimpleDictProtocol
import networkx as nx


class Node(Versioned, SimpleDictProtocol):
    version: Final = 1.0
    __class_name = 'Node'  # Can't find class name during instantiation of the class
    ATTRIB_MAP = {'id': dunder_name('id', __class_name),
                  'display_name': dunder_name('display_name', __class_name),
                  'parent': dunder_name('parent', __class_name),
                  'children': dunder_name('children', __class_name),
                  'x': dunder_name('x', __class_name),
                  'y': dunder_name('y', __class_name),
                  'theta': dunder_name('theta', __class_name),
                  }

    CLASS_ATTRIB_MAP = {'version': 'version'}

    def __init__(self, id: str, x: float, y: float, theta: float = 0.0, display_name: str = None, attr: dict = None):
        self.__x: float = x
        self.__y: float = y
        self.__theta: float = theta
        self.__id: str = str(id)  # Make sure to convert the ID to a string
        self.__display_name: Optional[str] = display_name

        # The following are set by the manager
        self.edges: Dict[Node, Edge] = dict()
        self.reverse_edges: Dict[Node, Edge] = dict()
        self.area: Optional[Area] = None
        self.__graph: Optional[nx.DiGraph] = None

        # Placeholders
        self.__parent: Optional[Node] = None
        self.__children: Dict[str, Node] = dict()

    def set_graph(self, graph: nx.DiGraph):
        self.__graph = graph

    def set_parent(self, parent, override_parent=False):
        if self.__parent is None and not override_parent:
            raise ParentExistsNodeError(
                'Node "{}" is already the child of "{}", cannot add new parent'.format(self.__id, parent.__id))
        self.__parent = parent

    def add_child(self, child: Node, override_parent: bool = False):
        if child.__id in self.__children:
            raise DuplicateChildNodeError(
                'Node "{}" has already been added to the parent "{}"'.format(child.__id, self.__id))

        # This can throw an error
        child.set_parent(self, override_parent=override_parent)

        # Only set the child after all checks are done
        self.__children[child.__id] = child

    def add_children(self, children, override_parent=False):
        for child in children:
            self.add_child(child, override_parent=override_parent)

    def add_edge(self, edge):
        """
        Add the directed edge, the end point is used as the key
        :param edge: Directed Edge that starts at this node
        :return:
        """
        if edge.end not in self.edges:
            self.edges[edge.end] = edge
        else:
            raise ParallelEdgeError(
                'Node {} already contains an edge to {}. '
                'Current edge: {}, edge to add: {}'.format(self.__id, edge.end,
                                                           self.edges[edge.end],
                                                           edge))

    def add_reverse_edge(self, edge: Edge) -> None:
        """
        Adds the reverse edge, i.e. this is the endpoint of the edge
        :param edge: Directed Edge that ends on this node
        :return:
        """
        if edge.start not in self.reverse_edges:
            self.reverse_edges[edge.start] = edge
        else:
            raise ParallelEdgeError('Node {} already contains the reverse edge to {}. '
                                    'Current edge: {}, edge to add: {}'.format(self.__id, edge.end,
                                                                               self.edges[edge.end], edge))

    @property
    def neighbours(self):
        return self.graph.neighbors(self)

    def to_simple_dict(self):
        attribs = dict()

        for k, v in self.__class__.CLASS_ATTRIB_MAP.items():
            attribs[k] = getattr(self.__class__, v)

        for k, v in self.__class__.ATTRIB_MAP.items():
            val = getattr(self, v)

            attribs[k] = to_simple(val)

        return {self.__class__.__name__: attribs}

    @classmethod
    def from_dict(cls, attrib_dict):
        if cls.__name__ in attrib_dict:
            data = attrib_dict[cls.__name__]
        else:
            raise ValueError('attrib_dict must contain one element with the key: {}'.format(cls.__name__))

        if 'version' in data:
            if not cls.validate_version(data['version']):
                raise Exception('Version is incorrect, got {} expected {}'.format(data['version'], cls.version))
        else:
            raise Exception('No version detected')

        # Manually set all the required attributes
        new: Node = cls.__new__(cls)
        for k, attr_name in cls.ATTRIB_MAP.items():
            val = data[k]
            try:
                setattr(new, attr_name, val)
            except Exception as e:
                print(e)

        # TODO: Check if attributes haven't been assigned from the dict

        return new

    @property
    def parent(self) -> Optional[Node]:
        return self.__parent

    @property
    def has_children(self):
        return len(self.__children) == 0

    @property
    def id(self):
        return self.__id

    @property
    def display_name(self):
        if self.__display_name is None:
            return self.__id
        else:
            return self.__display_name

    @property
    def graph(self):
        if self.__graph is None:
            raise Exception('Graph has not been set, need to add node to a graph_manager or call set_graph')
        return self.__graph

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    @property
    def theta(self):
        return self.__theta

    @property
    def pose(self):
        return PoseLike(self.__x, self.__y, self.__theta)

    @property
    def pos(self):
        return self.__x, self.__y

    def dist(self, pose: Union[PoseLike, Node]):
        return np.linalg.norm(np.array([pose.x - self.x, pose.y - self.y]), 2)

    def __hash__(self):
        return hash(self.__id)

    def __eq__(self, other: Node):
        equiv = [self.__x == other.__x,
                 self.__y == other.__y,
                 self.__theta == other.__theta,
                 self.__id == other.__id,
                 self.__display_name == other.__display_name,
                 self.__parent == other.__parent,
                 self.__children == other.__children
                 ]

        return all(equiv)

    def __repr__(self):
        return '{} - {}'.format(type(self), self.__id)

    def __str__(self):
        return '{}'.format(self.__id)


class Edge:
    """
    Assume only directional edges
    """

    def __init__(self, start, end):
        """
        When created, adds edge to the start node
        :param start: Starting node, edge will be added to start.edges
        :param end: Ending node
        :param behaviours: Placeholder for potential future behaviours
        """
        self.__start: Node = start
        self.__end: Node = end
        self.start.add_edge(self)

    @property
    def nodes(self):
        return self.start, self.end

    @property
    def start(self):
        return self.__start

    @property
    def end(self):
        return self.__end

    @property
    def nx_edge(self):
        return self.__start, self.__end

    def __hash__(self):
        return hash('{}_{}'.format(self.start.id, self.end.id))

    def __repr__(self):
        return '{}: {}-{}'.format(type(self), self.start.id, self.end.id)
