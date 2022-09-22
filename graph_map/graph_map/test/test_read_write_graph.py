import json
import os
import unittest

from graph_map.node import Node, Edge
from graph_map.node_graph_manager import NodeGraphManager


def make_graph():
    g = NodeGraphManager()

    # Make Nodes in 2x2 grid
    # 0 -> 1
    # | \
    # 3 - 2
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
                                  "x": 0,
                                  "y": 0,
                                  "theta": 0.0}},
                         "id": "0"},
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "1",
                                  "display_name": "node_1",
                                  "parent": None,
                                  "children": {},
                                  "x": 1,
                                  "y": 0,
                                  "theta": 0.0}},
                         "id": "1"},
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "2",
                                  "display_name":
                                      "node_2",
                                  "parent": None,
                                  "children": {},
                                  "x": 1,
                                  "y": 1,
                                  "theta": 0.0}},
                         "id": "2"},
                     {"_node": {
                         "Node": {"version": 1.0,
                                  "id": "3",
                                  "display_name":
                                      "node_3",
                                  "parent": None,
                                  "children": {},
                                  "x": 0,
                                  "y": 1,
                                  "theta": 0.0}},
                         "id": "3"}],
                 "adjacency": [[{"id": "1"}, {"id": "3"}, {"id": "2"}], [], [{"id": "3"}, {"id": "0"}],
                               [{"id": "0"}, {"id": "2"}]]}

    return g, json_dict


class ReadWriteTest(unittest.TestCase):

    def test_read_write_graph(self):
        gm1, json_dict = make_graph()
        test_path = './test.json'
        try:
            gm1.write_graph(test_path)

            with open(test_path, 'r') as fp:
                new_json_dict = json.load(fp)

            self.assertDictEqual(new_json_dict, json_dict)

            gm2 = NodeGraphManager.read_graph(test_path)

            self.assertEqual(gm1, gm2)
        finally:
            try:
                os.remove(test_path)  # Delete the test file after
            except Exception:
                pass  # Don't care if this fails


if __name__ == '__main__':
    unittest.main()
