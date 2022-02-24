import json
import logging
import networkx as nx
import matplotlib.pyplot as plt
from typing import Dict, Optional, Iterable, List, Tuple, Final
from shapely.geometry import Point, Polygon
from geometry_msgs.msg import Polygon as PolygonMsg

from graph_map.util import SimpleDictProtocol, PoseLike
from graph_map.graph_exceptions import AreaError, UnknownAreaError, InconsistentTreeAreaError
from graph_map.util import to_simple, dunder_name, Versioned
from graph_map.msg import Zone as ZoneMsg
from graph_map.msg import Region as RegionMsg
# from graph_map.node import Node

from std_msgs.msg import ColorRGBA

logger = logging.getLogger(__name__)


class Color(SimpleDictProtocol):
    # Placeholder
    def __init__(self, r=0.5, g=0.5, b=0.5, a=0.5):
        self.r: float = r
        self.g: float = g
        self.b: float = b
        self.a: float = a

    @classmethod
    def from_dict(cls, attrib_dict) -> 'Color':
        return Color(r=attrib_dict['r'], g=attrib_dict['g'], b=attrib_dict['b'], a=attrib_dict['a'])

    def to_msg(self) -> ColorRGBA:
        return ColorRGBA(r=self.r, g=self.g, b=self.b, a=self.a)

    def to_simple_dict(self) -> Dict:
        return {'r': self.r, 'g': self.g, 'b': self.b, 'a': self.a}

    def __eq__(self, o: object) -> bool:
        assert(isinstance(o, Color))

        equiv = [
                    self.r == o.r,
                    self.g == o.g,
                    self.b == o.b,
                    self.a == o.a
                ]

        return all(equiv)


class Region(SimpleDictProtocol):
    __class_name = 'Region'  # Can't find class name during instantiation of the class
    ATTRIB_MAP = {'points': 'points',
                  'color': dunder_name('color', __class_name)}

    def __init__(self, points: Iterable[Iterable[float]], color: Color = Color()):
        self.__polygon = Polygon(points).simplify(1e-4)  # simplify can fix some issues like duplicate points
        self.__color = color

    @classmethod
    def from_dict(cls, attrib_dict):
        # Manually set all the required attributes
        new: Region = cls.__new__(cls)
        new.__init__(points=attrib_dict['points'], color=Color.from_dict(attrib_dict['color']))

        return new

    @classmethod
    def from_msg(cls, msg: RegionMsg):
        # Manually set all the required attributes
        new: Region = cls.__new__(cls)
        new.__init__(
            points=[(p.x, p.y) for p in msg.polygon.points],
            color=Color(r=msg.color.r, g=msg.color.g, b=msg.color.b, a=msg.color.a)
        )
        return new

    @property
    def points(self) -> List[Tuple[float, float]]:
        return list(self.__polygon.exterior.coords)

    @property
    def polygon(self) -> Polygon:
        return self.__polygon

    @property
    def color(self) -> Color:
        return self.__color

    def set_color(self, color):
        self.__color = color

    def is_in(self, pose: PoseLike):
        point = Point(pose.x, pose.y)
        return self.__polygon.intersects(point)

    def __repr__(self) -> str:
        return repr(self.points)

    def __eq__(self, o: object) -> bool:
        return self.points == o.points and self.color == o.color

    def to_simple_dict(self):
        attribs = dict()

        for k, v in self.__class__.ATTRIB_MAP.items():
            val = getattr(self, v)

            attribs[k] = to_simple(val)

        return attribs

    def to_msg(self):
        return RegionMsg(
            polygon=PolygonMsg(points=[Point(point[0], point[1], 0) for point in self.points]),
            color=ColorRGBA(r=self.color.r, g=self.color.g, b=self.color.b, a=self.color.a)
        )

    def plot(self, ax=None):
        if ax is None:
            plt.plot(*self.polygon.exterior.xy, color=(self.color.r, self.color.g, self.color.b, self.color.a))
        else:
            ax.plot(*self.polygon.exterior.xy, color=(self.color.r, self.color.g, self.color.b, self.color.a))


class Area(Versioned, SimpleDictProtocol):
    # TODO Need to recursively generate the total area
    # TODO Need to check for membership in area
    # TODO Should have warnings for areas that are not very convex (to some measure)
    # TODO Should give an error report for overlapping areas

    version: Final = 1.0
    __class_name = 'Area'  # Can't find class name during instantiation of the class
    ATTRIB_MAP = {'id': dunder_name('id', __class_name),
                  'display_name': dunder_name('display_name', __class_name),
                  'regions': dunder_name('regions', __class_name),
                  }

    CLASS_ATTRIB_MAP = {'version': 'version'}

    def __init__(self, id: str, regions: Iterable[Region] = None, display_name: Optional[str] = None):
        """
        Abstraction of an area, provides convenient access to pose membership.
        :param name: Unique name of the area or ID
        :param area_units: Iterable[Region] describing the physical location of the areas
        :param children: List of Areas that are strict subsets of this Area. Children must not have overlaps
        :param parent: Area that this is a strict subset of
        """
        self.__id: str = id
        self.__display_name: Optional[str] = display_name
        if regions is None:
            self.__regions = list()
        else:
            self.__regions: List[Region] = list(regions)

        # Used after init
        self.__tree: Optional[nx.DiGraph] = None
        self.nodes: List = list()

    def __hash__(self):
        return hash(self.__id)

    def __eq__(self, other: 'Area'):
        assert(isinstance(other, Area))

        equiv = [
                    self.id == other.id,
                    self.display_name == other.display_name,
                    self.regions == other.regions
                ]

        return all(equiv)

    def __repr__(self):
        return '{}: {}'.format(type(self), self.__id)

    def __str__(self):
        return self.__id

    def to_simple_dict(self):
        attribs = dict()

        for k, v in self.__class__.CLASS_ATTRIB_MAP.items():
            attribs[k] = getattr(self.__class__, v)

        for k, v in self.__class__.ATTRIB_MAP.items():
            val = getattr(self, v)

            attribs[k] = to_simple(val)

        return attribs

    @classmethod
    def from_dict(cls, attrib_dict):
        if 'version' in attrib_dict:
            if not cls.validate_version(attrib_dict['version']):
                raise Exception('Version is incorrect, got {} expected {}'.format(attrib_dict['version'], cls.version))
        else:
            raise Exception('No version detected')

        # Manually set all the required attributes
        for k in cls.ATTRIB_MAP.keys():
            if k not in attrib_dict.keys():
                raise Exception('{} not found in attribute dictionary'.format(k))

        new: Area = cls.__new__(cls)
        new.__init__(id=attrib_dict['id'],
                     regions=[Region.from_dict(r) for r in attrib_dict['regions']],
                     display_name=attrib_dict['display_name'])

        # TODO: Check if attributes haven't been assigned from the dict

        return new

    @property
    def id(self) -> str:
        """
        Area ID
        :return: id string
        """
        return self.__id

    @property
    def display_name(self):
        if self.__display_name is None:
            return self.__id
        else:
            return self.__display_name

    @property
    def level(self) -> int:
        """
        Area's level
        :return: level
        """
        if len(self.children) == 0:
            return 1
        else:
            children_levels = set()
            for child in self.children:
                assert(child is not self)
                children_levels.add(child.level)
            if len(children_levels) > 1:
                # Children have different levels. Something is wrong with our tree.
                raise InconsistentTreeAreaError("{}'s children have different levels: {}"
                                                .format(self.id, children_levels))

            return children_levels.pop() + 1

    @property
    def tree(self):
        return self.__tree

    def set_graph(self, graph: nx.Graph):
        self.__tree = graph

    def ancestor(self, generations: int = 1) -> Optional['Area']:
        """
        Get ancestor area
        :return: parent area
        """
        if self.tree is None:
            logger.error('Tree must be set before getting parent')
            return None

        if generations == 0:
            return self

        generations = max(1, generations)
        current_area: 'Area' = self
        for i in range(generations):
            current_area = next(current_area.tree.predecessors(current_area), None)

        return current_area

    @property
    def parent(self) -> Optional['Area']:
        """
        Get parent area.
        :return: parent area
        """
        return self.ancestor(generations=1)

    @parent.setter
    def parent(self, parent: 'Area') -> None:
        """
        Set parent area by creating the appropriate edge
        :return: parent area
        """
        if self.tree is None:
            raise AreaError('Tree must be set before getting parent')

        if parent not in self.tree.nodes:
            raise UnknownAreaError('Tree does not contain the parent area {}'.format(parent.id))

        if self not in self.tree.nodes:
            raise UnknownAreaError('Tree does not contain the child area {}'.format(self.id))

        if self.parent is None:
            self.tree.add_edge(parent, self)
        elif self.parent is parent:
            pass
        else:
            logger.warning("{}'s parent is already {}. Unsetting and changing to {}".format(self.id, self.parent,
                                                                                            parent.id))
            self.tree.remove_edges_from([(pre, self) for pre in self.tree.predecessors(self)])
            self.tree.add_edge(parent, self)

    @property
    def children(self) -> List['Area']:
        return list(self.tree.successors(self))

    def is_in(self, pose: PoseLike) -> bool:
        """
        Check if a pose or position is in the area
        :param pose: Vector 2/3 or contains x,y
        :return:
        """
        for region in self.regions:
            if region.is_in(pose):
                return True

        return False

    def validate(self):
        """
        Run validation on the area and return a list of warnings/errors but does not fail the creation of this object
        :return: Dict with issue levels and list of issues
        """
        # TODO create a set of rules for area validation, maybe this has to be done at the "area unit" level
        issues = {'Warning': [], 'Error': []}

        try:
            # Check area has the tree set correctly
            is_in_tree = False
            for node in self.tree.nodes:
                if self is node:
                    is_in_tree = True
                    break
            if not is_in_tree:
                issues['Error'].append("Area {} is not in its own tree. Hint: This is bad.".format(self.id))

            # Check node levels are consistent. The level function will do this check.
            if self.level > 1 and len(self.__regions) >= 1:
                issues['Error'].append("Area {} is level {} but has regions".format(self.id, self.level))

            # Check loner areas
            if self.parent is None and len(self.children) == 0:
                issues['Warning'].append("Area {} has no children or parent".format(self.id))
        except Exception:
            pass  # pass so that the full report can still be generated

        return issues

    @property
    def regions(self) -> List[Region]:
        """
        Returns a collection of the basic area structures by recursively searching child areas.
        :return: iterable of area units
        """
        regions = list()
        if self.tree is None or len(self.children) == 0:
            return self.__regions
        else:
            for child in self.children:
                regions += child.regions
            return regions

    def add_region(self, region: Region):
        self.__regions.append(region)

    def plot(self, ax=None) -> None:
        for region in self.regions:
            region.plot(ax=ax)


class Zone(Versioned, SimpleDictProtocol):

    version: Final = 1.0
    __class_name = 'Zone'  # Can't find class name during instantiation of the class
    ATTRIB_MAP = {'id': dunder_name('id', __class_name),
                  'display_name': dunder_name('display_name', __class_name),
                  'regions': dunder_name('regions', __class_name),
                  'drivable': dunder_name('drivable', __class_name),
                  'cost': dunder_name('cost', __class_name),
                  'attr': dunder_name('attr', __class_name),
                  }

    CLASS_ATTRIB_MAP = {'version': 'version'}

    def __init__(self,
                 id: str,
                 regions: Iterable[Region] = None,
                 display_name: Optional[str] = None,
                 drivable: bool = True,
                 cost: float = 0.0,
                 attr: Optional[dict] = None):
        """
        Abstraction of an area, provides convenient access to pose membership.
        :param name: Unique name of the area or ID
        :param area_units: Iterable[Region] describing the physical location of the areas
        :param children: List of Areas that are strict subsets of this Area. Children must not have overlaps
        :param parent: Area that this is a strict subset of
        """
        self.__id: str = id
        self.__display_name: Optional[str] = display_name
        if regions is None:
            self.__regions = list()
        else:
            self.__regions: List[Region] = list(regions)

        assert(isinstance(drivable, bool))
        self.__drivable = drivable
        assert(isinstance(cost, float))
        self.__cost = cost
        if isinstance(attr, dict):
            self.__attr = attr
        else:
            self.__attr = dict()

    @classmethod
    def from_msg(cls, msg: ZoneMsg):
        attr = json.loads(msg.attr)

        new: Zone = cls.__new__(cls)
        new.__init__(id=msg.id,
                     regions=[Region.from_msg(region) for region in msg.regions],
                     display_name=msg.display_name,
                     drivable=msg.drivable,
                     cost=msg.cost,
                     attr=attr)
        return new

    def __eq__(self, other: 'Zone'):
        assert(isinstance(other, Zone))

        equiv = [
                    self.id == other.id,
                    self.display_name == other.display_name,
                    self.regions == other.regions,
                    self.drivable == other.drivable,
                    self.cost == other.cost,
                    self.attr == other.attr
                ]

        return all(equiv)

    def __repr__(self):
        return '{}: {}'.format(type(self), self.__id)

    def __str__(self):
        return self.__id

    def is_in(self, pose: PoseLike) -> bool:
        """
        Check if a pose or position is in the area
        :param pose: Vector 2/3 or contains x,y
        :return:
        """
        for region in self.regions:
            if region.is_in(pose):
                return True

        return False

    def to_simple_dict(self):
        attribs = dict()

        for k, v in self.__class__.CLASS_ATTRIB_MAP.items():
            attribs[k] = getattr(self.__class__, v)

        for k, v in self.__class__.ATTRIB_MAP.items():
            val = getattr(self, v)

            attribs[k] = to_simple(val)

        return attribs

    def to_msg(self):
        msg = ZoneMsg()
        msg.id = self.id
        msg.display_name = self.display_name
        msg.drivable = self.drivable
        msg.cost = self.cost
        msg.regions = [r.to_msg() for r in self.regions]
        msg.attr = json.dumps(self.attr)

        return msg

    @classmethod
    def from_dict(cls, attrib_dict):
        if 'version' in attrib_dict:
            if not cls.validate_version(attrib_dict['version']):
                raise Exception('Version is incorrect, got {} expected {}'.format(attrib_dict['version'], cls.version))
        else:
            raise Exception('No version detected')

        # Manually set all the required attributes
        for k in cls.ATTRIB_MAP.keys():
            if k not in attrib_dict.keys():
                raise Exception('{} not found in attribute dictionary'.format(k))

        new: Zone = cls.__new__(cls)
        new.__init__(id=attrib_dict['id'],
                     regions=[Region.from_dict(r) for r in attrib_dict['regions']],
                     display_name=attrib_dict['display_name'],
                     drivable=attrib_dict['drivable'],
                     cost=attrib_dict['cost'],
                     attr=attrib_dict['attr'])

        # TODO: Check if attributes haven't been assigned from the dict

        return new

    @property
    def id(self) -> str:
        """
        Zone ID
        :return: id string
        """
        return self.__id

    @property
    def display_name(self):
        if self.__display_name is None:
            return self.__id
        else:
            return self.__display_name

    @property
    def regions(self) -> List[Region]:
        """
        :return: regions
        """
        return self.__regions

    @property
    def behaviour(self) -> str:
        return self.__behaviour

    @property
    def drivable(self) -> bool:
        return self.__drivable

    @property
    def cost(self) -> float:
        return self.__cost

    @property
    def attr(self) -> dict:
        return self.__attr

    def add_region(self, region: Region):
        self.__regions.append(region)

    def plot(self, ax=None) -> None:
        for region in self.regions:
            region.plot(ax=ax)
