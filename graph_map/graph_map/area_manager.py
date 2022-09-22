import logging

from typing import Dict, Optional, Tuple, Sequence, Iterable, List
from .graph_exceptions import AreaError, DuplicateAreaError, NotATreeAreaError
from .nx_graph_manager import NxGraphManager

from .area import Area

import networkx as nx
logger = logging.getLogger(__name__)


class AreaManager(NxGraphManager):
    NODE_KEY = '_area'

    def __init__(self):
        self.tree: nx.DiGraph = nx.DiGraph()  # It's a tree
        self.tree_index_map: Dict[int, str] = dict()

        # cached data
        self.__areas: Optional[Dict[Area, Dict]] = None
        self.__area_ids: Optional[Dict[str, Area]] = None

    def _dirty(func):
        def wrapper(self: 'AreaManager', *args, **kwargs):
            self.__clear_cache()
            return func(self, *args, **kwargs)

        return wrapper

    def _validate_tree(func):
        def wrapper(self, *args, **kwargs):
            result = func(self, *args, **kwargs)
            if not nx.is_forest(self.tree):
                raise NotATreeAreaError
            return result

        return wrapper

    @_dirty
    @_validate_tree
    def add_area(self, area: Area, attr=None):
        if attr is None:
            attr = dict()

        if area not in self.tree.nodes:
            self.tree.add_node(area, **attr)
            area.set_graph(self.tree)
        else:
            raise DuplicateAreaError('Area {} already exists'.format(area.id))

    @_dirty
    @_validate_tree
    def add_areas(self, areas: Iterable[Area]):
        duplicates = list()
        # We aren't using self.tree.add_nodes_from() so that we can check for duplicates
        for area in areas:
            try:
                if isinstance(area, Area):
                    self.add_area(area)
                else:
                    self.add_area(*area)  # Unpack (area, attr)
            except DuplicateAreaError:
                duplicates.append(area.id)

        if duplicates:
            raise DuplicateAreaError('Duplicate Areas found: {}'.format(duplicates))

    @_dirty
    @_validate_tree
    def add_edge(self, parent: Area, child: Area) -> None:
        # Using this setter will guarantee tree structure is maintained
        child.parent = parent

    @_dirty
    @_validate_tree
    def add_edge_by_id(self, start_id, end_id):
        # Needed for external interfaces which will only have the IDs and names
        raise NotImplementedError

    @_dirty
    @_validate_tree
    def add_edges(self, edges: Iterable[Tuple[Area, Area]]):
        """
        Adds edges from a sequence of tuples containing (parent: Area, child: Area)
        :param edges: Iterable of tuples of (parent, child)
        :return:
        """
        for edge in edges:
            try:
                self.add_edge(edge[0], edge[1])
            except Exception:
                # Placeholder for known exceptions
                raise

    @_dirty
    @_validate_tree
    def remove_edge(self, parent, child):
        # NetworkX does raise an exception if the edge does not exist
        self.tree.remove_edge(parent, child)

    @_dirty
    @_validate_tree
    def remove_edges(self):
        raise NotImplementedError

    @_dirty
    @_validate_tree
    def remove_area(self):
        raise NotImplementedError

    @_dirty
    @_validate_tree
    def remove_areas(self):
        raise NotImplementedError

    @_dirty
    def modify_area_attrs(self, area: Area, attr: Dict, clear: bool = False):
        attr_dict = self.tree[area]
        if clear:
            attr_dict.clear()
        attr_dict.update(attr)

    def shortest_path(self, start, end):
        # Note that we can also compute all_pairs_shortest_path if we want to query multiple solutions
        return nx.shortest_path(self.tree, source=start, target=end, weight='weight')

    def subtree(self, areas):
        return nx.subgraph(self.tree, areas)

    def write_graph(self, filepath):
        self._write_graph(filepath, AreaManager.NODE_KEY)

    def to_json(self):
        return self._to_json(AreaManager.NODE_KEY)

    @classmethod
    def read_graph(cls, filepath):
        """
                Reads a networkX graph saved using json_graph.adjacency_data
                :param filepath: Path to graph data
                :return: AreaManager with the graph loaded from the file
                """
        am = cls._read_graph(filepath, AreaManager.NODE_KEY, Area)

        issues: Dict[str, List] = am.validate()

        for error in issues['Error']:
            logger.error(error)

        for warning in issues['Warning']:
            logger.warning(warning)

        return am

    @classmethod
    def from_jsons(cls, json_str):
        """
                Reads a networkX graph saved using json_graph.adjacency_data
                :param json_str: Graph data in JSON string format
                :return: AreaManager with the graph loaded from the file
                """
        am = cls._from_jsons(json_str, AreaManager.NODE_KEY, Area)

        issues: Dict[str, List] = am.validate()

        for error in issues['Error']:
            logger.error(error)

        for warning in issues['Warning']:
            logger.warning(warning)

        return am

    def get_areas(self, level: Optional[int] = 1):
        """
        Returns a dictionary of areas (keys) and their attributes as a dict (values)
        :return:
        """
        if self.__areas is None:
            self.__areas = dict(self.tree.nodes)
            self.__area_ids = None

        if level is None:
            return self.__areas
        else:
            return {area: att for area, att in self.__areas.items() if area.level == level}

    @property
    def areas(self):
        """
        Returns a dictionary of areas and their attributes
        :return:
        """
        return self.get_areas(level=None)

    def get_nodes(self):
        return self.areas

    def get_graph(self):
        return self.tree

    def set_graph(self, graph):
        self.tree = graph

    def get_area_ids(self, level: Optional[int] = None):
        """
        Returns a dictionary of area ids and their areas
        :return:
        """
        if self.__area_ids is None:
            self.__area_ids = {area.id: area for area in self.areas.keys()}

        if level is None:
            return self.__area_ids
        else:
            return {id: area for id, area in self.__area_ids.items() if area.level == level}

    @property
    def area_ids(self):
        """
        Returns a dictionary of area ids and their areas
        :return:
        """
        return self.get_area_ids()

    def area_by_id(self, id: str):
        """
        Returns an area by the id
        :return:
        """
        if self.__area_ids is None:
            self.__area_ids = {area.id: area for area in self.areas.keys()}

        return self.__area_ids[id]

    def validate(self):
        """
        Run validation on the area and return a list of warnings/errors but does not fail the creation of this object
        :return: Dict with issue levels and list of issues
        """
        issues = {'Warning': [], 'Error': []}
        # Check arborescence (directed tree with maximum in-degree of 1)
        if nx.is_forest(self.tree) is False:
            issues['Error'].append('Area graph is not a forest (there are cycles)')
        if nx.is_arborescence(self.tree) is False:
            issues['Warning'].append('Area graph is not aborescent')

        # Getting the level will perform a check of inconsistent childrens (ie. children with different levels)
        for area in self.area_ids.values():
            try:
                area_issues = area.validate()
                issues['Error'] += area_issues['Error']
                issues['Warning'] += area_issues['Warning']
            except AreaError as e:
                issues['Error'].append(e)

        return issues

    def __eq__(self, other):
        # This does not test attrs
        if other.tree.edges != self.tree.edges:
            return False

        # This also tests attrs
        snd = dict(self.tree.nodes)
        ond = dict(other.tree.nodes)

        if snd != ond:
            print('Snd: {}\nOnd: {}'.format(snd, ond))

        if other.tree.nodes != self.tree.nodes:
            return False

        return True

    def __contains__(self, item):
        if isinstance(item, Area):
            return item in self.areas
        elif isinstance(item, str):
            # Assume it must be an ID
            return item in self.area_ids
        elif isinstance(item, Sequence):
            if len(item) == 2:
                # (start, end (, attr))
                if isinstance(item[0], str) and isinstance(item[1], str):
                    # Areas represented with ids
                    return item in self.tree.edges
                elif isinstance(item[0], Area) and isinstance(item[1], Area):
                    # Areas represented with Areas
                    return item in self.tree.edges
                else:
                    raise ValueError('Sequence input assumed to be an edge. Start and End areas must be of type '
                                     'Area or str, got ({}, {})'.format(type(item[0]), type(item[1])))
            else:
                raise ValueError('Sequence input is assumed to be an edge of form '
                                 '\'(start, end (, attr)\'. Item must be length 2 or 3 got {}.'.format(len(item)))
        else:
            return NotImplementedError('Unknown input type: {}'.format(type(item)))

    def __clear_cache(self):
        self.__areas = None
        self.__area_ids = None
