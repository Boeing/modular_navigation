from typing import Dict, Iterable, Protocol, Union
from abc import abstractmethod
import networkx as nx

from logging import getLogger

from matplotlib import pyplot as plt
from networkx.drawing.nx_pydot import graphviz_layout

from geometry_msgs.msg import Transform, Pose

from math6d.geometry.quaternion import Quaternion
from math6d.geometry.vector3 import Vector3

logger = getLogger(__name__)


class PoseLike:
    # Placeholder
    def __init__(self, x, y, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
    pass

    def __str__(self):
        return str((self.x, self.y, self.theta))

    @classmethod
    def from_msg(cls, msg: Union[Transform, Pose]):
        if isinstance(msg, Transform):
            return PoseLike(msg.translation.x,
                            msg.translation.y,
                            Quaternion.from_msg(msg.rotation).to_euler()[2])
        elif isinstance(msg, Pose):
            return PoseLike(msg.position.x,
                            msg.position.y,
                            Quaternion.from_msg(msg.orientation).to_euler()[2])
        else:
            raise ValueError('Unknown message type. Should be Transform or Pose only')

    def to_msg(self) -> Pose:
        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y
        msg.orientation = Quaternion.from_axis_angle(Vector3(0, 0, 1.0), self.theta)
        return msg


def to_simple(val):
    if val.__class__.__module__ != 'builtins':
        if hasattr(val, 'to_simple_dict'):
            out = val.to_simple_dict()
        else:
            raise NotImplementedError('Type {} does not implement SimpleDictProtocol'.format(type(val)))
    else:
        # Handle containers with arbitrary types
        if isinstance(val, Dict):
            # Expand into a dict, applying `to_simple` to both keys and values
            out = {to_simple(k): to_simple(v) for k, v in val.items()}
        elif isinstance(val, Iterable) and not isinstance(val, str):
            # Assume we can reconstruct the container using the type
            # Since this is used for JSON conversion, convert everything to lists as they'll be converted to lists
            # anyway
            out = list(map(to_simple, val))
        else:
            out = val  # Builtin primitive

    return out


def dunder_name(name, class_name):
    return '_{}__{}'.format(class_name, name)


class Versioned:
    @property
    @abstractmethod
    def version(self):
        pass

    @classmethod
    def validate_version(cls, version):
        # Make sure to check the Class's instance of version
        return cls.version == version


class SimpleDictProtocol(Protocol):
    """
    Protocol defining a method to convert the class to a dictionary with only primitives
    Protocols are statically checkable, need runtim checkability
    """

    @abstractmethod
    def to_simple_dict(self):
        raise NotImplementedError

    @abstractmethod
    def from_dict(self, attrib_dict: Dict):
        raise NotImplementedError


class NxGraphManagerNode:
    @property
    @abstractmethod
    def nodes(self):
        raise NotImplementedError

    @abstractmethod
    def set_graph(self, graph: nx.Graph):
        raise NotImplementedError


def draw_node_graph(graph: nx.Graph, positions: bool, show: bool = True):
    label_dict = {node: node.display_name for node in graph.nodes}
    pos_dict = None
    if positions:
        pos_dict = {node: (node.x, node.y) for node in graph.nodes}

    nx.draw(graph, pos_dict, labels=label_dict, with_labels=True)
    if show:
        plt.show()


def draw_area_graph(graph: nx.Graph, show: bool = True):
    label_dict = {area: area.display_name for area in graph.nodes}
    pos = graphviz_layout(graph, prog="dot")
    nx.draw(graph, pos, labels=label_dict, with_labels=True)
    plt.show()


def closest_node_in_subgraph(pose: PoseLike, graph: nx.Graph):
    dists = {node: node.dist(pose) for node in graph.nodes}
    k = min(dists, key=dists.get)
    return k, dists[k]
