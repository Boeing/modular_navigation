import unittest

from graph_map.node import Node, Edge
from graph_map.node_graph_manager import NodeGraphManager
from graph_map.area import Area, Region, PoseLike
from graph_map.area_manager import AreaManager


def make_graph():
    g = NodeGraphManager()

    # Make Nodes in 2x2 grid
    # 3 - 2
    # | /
    # 0 -> 1

    positions = [(0, 0), (1, 0), (1, 1), (0, 1)]
    nodes = dict()
    for i, (x, y) in enumerate(positions):
        new_node = Node(id=str(i), x=x, y=y, display_name='node_{}'.format(i))
        g.add_node(new_node)
        nodes[str(i)] = new_node

    edge_ids = [(0, 1),
                (0, 3), (3, 0),
                (3, 2), (2, 3),
                (0, 2), (2, 0)
                ]

    for start, end in edge_ids:
        new_edge = Edge(nodes[str(start)], nodes[str(end)])
        g.add_edge(new_edge)

    json_dict = {"directed": True,
                 "multigraph": False,
                 "graph": [],
                 "nodes": [
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "0",
                                  "display_name": "node_0",
                                  "parent": None,
                                  "children": {},
                                  "area": None,
                                  "x": 0,
                                  "y": 0,
                                  "behaviours": None}},
                         "id": "0"},
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "1",
                                  "display_name": "node_1",
                                  "parent": None,
                                  "children": {},
                                  "area": None,
                                  "x": 1,
                                  "y": 0,
                                  "behaviours": None}},
                         "id": "1"},
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "2",
                                  "display_name":
                                      "node_2",
                                  "parent": None,
                                  "children": {},
                                  "area": None,
                                  "x": 1,
                                  "y": 1,
                                  "behaviours": None}},
                         "id": "2"},
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "3",
                                  "display_name":
                                      "node_3",
                                  "parent": None,
                                  "children": {},
                                  "area": None,
                                  "x": 0,
                                  "y": 1,
                                  "behaviours": None}},
                         "id": "3"}],
                 "adjacency": [[{"id": "1"}, {"id": "3"}, {"id": "2"}], [], [{"id": "3"}, {"id": "0"}],
                               [{"id": "0"}, {"id": "2"}]]}

    return g, json_dict


class NodeGraphTest(unittest.TestCase):

    # TODO More basic unit tests

    # Make Nodes in 2x2 grid and areas enclosing them
    #  _________________
    # | 3      | 2      |
    # |        |        |
    # | 0      | 1      |
    # |________|________|
    #   Area 1   Area 2
    def test_nearest_node(self):
        gm, _ = make_graph()

        # Encloses nodes 0 and 3
        points_1 = [[-1.0, -1.0], [-1.0, 2.0], [0.9, 2.0], [0.9, -1.0]]
        regions_1 = [Region(points_1)]
        area_1 = Area(id='area_1', regions=regions_1)

        # Encloses nodes 1 and 2
        points_2 = [[0.9, -1.0], [0.9, 2.0], [2.0, 2.0], [2.0, -1.0]]
        regions_2 = [Region(points_2)]
        area_2 = Area(id='area_2', regions=regions_2)

        am = AreaManager()
        am.add_areas([area_1, area_2])

        root_area = Area(id='root')
        am.add_area(root_area)
        area_1.parent = root_area
        area_2.parent = root_area

        gm.set_area_manager(am)
        self.assertEqual(gm.closest_node(PoseLike(0.0, 0.0))[0].id, '0')
        self.assertEqual(gm.closest_node(PoseLike(0.5, 0.0))[0].id, '0')
        self.assertEqual(gm.closest_node(PoseLike(0.8, 0.0))[0].id, '0')

        self.assertEqual(gm.closest_node(PoseLike(0.0, 0.6))[0].id, '3')
        self.assertEqual(gm.closest_node(PoseLike(0.5, 0.6))[0].id, '3')
        self.assertEqual(gm.closest_node(PoseLike(0.8, 0.6))[0].id, '3')

        self.assertEqual(gm.closest_node(PoseLike(1.0, 0.0))[0].id, '1')
        self.assertEqual(gm.closest_node(PoseLike(1.0, 0.6))[0].id, '2')

        with self.assertRaises(ValueError, msg='Could not find Area enclosing pose'):
            gm.closest_node(PoseLike(3.0, 0.5))


if __name__ == '__main__':
    unittest.main()
