import os
import unittest
from typing import List, Dict, Set

from graph_map.area import Area, Region, Zone
from map_manager.dxf.extract import extract_obstacles, extract_bollards, extract_nodes, extract_edges, extract_areas, \
    extract_zones, extract_groups, load_doc, Group
from graph_map.node import Node, Edge


DXF_FILE = os.path.join(os.path.dirname(__file__), 'test.dxf')


class ExtractFloorplanTest(unittest.TestCase):

    def setUp(self) -> None:
        self.doc = load_doc(DXF_FILE)

    def test_extract_areas(self):
        areas: Dict[str, Area] = extract_areas(self.doc)

        expected_areas = {
            'ROOM_1': Area(id='ROOM_1',
                           regions=[Region([(4.8, 0.2), (4.8, 9.8), (0.2, 9.8), (0.2, 0.2)])],
                           display_name='Room 1')
        }

        self.assertEqual(len(areas), 1)
        self.assertEqual(areas['ROOM_1'].id, expected_areas['ROOM_1'].id)
        self.assertEqual(areas['ROOM_1'].display_name, expected_areas['ROOM_1'].display_name)
        for i in range(4):
            self.assertAlmostEqual(areas['ROOM_1'].regions[0].points[i][0],
                                   expected_areas['ROOM_1'].regions[0].points[i][0], delta=0.01)
            self.assertAlmostEqual(areas['ROOM_1'].regions[0].points[i][1],
                                   expected_areas['ROOM_1'].regions[0].points[i][1], delta=0.01)

    def test_extract_bollards(self):
        bollards: List[Dict] = extract_bollards(self.doc)

        self.assertEqual(len(bollards), 1)
        self.assertEqual(bollards[0]['radius'], 0.07)
        self.assertEqual(bollards[0]['center'], '2.5 2.5')

    def test_extract_graph(self):
        nodes: Dict[str, Node] = extract_nodes(self.doc)
        edges: Set[Edge] = extract_edges(self.doc, [node_tuple[0] for node_tuple in nodes.values()])
        self.assertIn('node_1', nodes.keys())
        self.assertIn('node_2', nodes.keys())
        self.assertIn('node_3', nodes.keys())

        # One directed edge and one undirected gives 3 edges
        self.assertEqual(len(edges), 3)

    def test_extract_groups(self):
        groups: List[Group] = extract_groups(self.doc)

        self.assertEqual(len(groups), 1)
        self.assertEqual(groups[0].id, 'test_group')
        self.assertEqual(groups[0].level, 2)
        self.assertEqual(groups[0].display_name, 'Test Group')
        self.assertEqual(groups[0].children.pop(), 'ROOM_1')

    def test_extract_obstacles(self):
        obstacles: List[Dict] = extract_obstacles(self.doc)
        self.assertEqual(len((obstacles[0]['links'][0]['points'])), 4)
        self.assertEqual(obstacles[0]['height'], 1.5)

    def test_extract_zones(self):
        zones: Dict[str, Zone] = extract_zones(self.doc)
        self.assertEqual(len(zones), 1)
        self.assertEqual(zones['SLOW']._Zone__display_name, 'SLOW')
        self.assertEqual(zones['SLOW']._Zone__id, 'SLOW')
        self.assertEqual(len(zones['SLOW']._Zone__regions[0].points), 5)


if __name__ == '__main__':
    unittest.main()
