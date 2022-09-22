import json
import os
import unittest

from graph_map.area import Area, Region
from graph_map.area_manager import AreaManager


def make_tree():
    am = AreaManager()

    area_1 = Area(id='area_1', regions=[Region([[0, 0], [0, 1], [1, 1], [1, 0]])])
    area_2 = Area(id='area_2', regions=[Region([[2, 0], [2, 1], [3, 1], [3, 0]])])
    root = Area(id='root')
    am.add_area(area_1)
    am.add_area(area_2)
    am.add_area(root)

    area_1.parent = root
    area_2.parent = root

    json_dict = {
        "directed": True,
        "multigraph": False,
        "graph": [],
        "nodes": [
            {
                "_area": {
                    "version": 1.0,
                    "id": "area_1",
                    "display_name": None,
                    "regions": [
                        {
                            "points": [
                                [0, 0],
                                [0, 1],
                                [1, 1],
                                [1, 0],
                                [0, 0]
                            ],
                            "color": {
                                'r': 0.5,
                                'b': 0.5,
                                'g': 0.5,
                                'a': 0.5
                            }
                        }
                    ]
                },
                "id": "area_1"
            },
            {
                "_area": {
                    "version": 1.0,
                    "id": "area_2",
                    "display_name": None,
                    "regions": [
                        {
                            "points": [
                                [2, 0],
                                [2, 1],
                                [3, 1],
                                [3, 0],
                                [2, 0]
                            ],
                            "color": {
                                'r': 0.5,
                                'b': 0.5,
                                'g': 0.5,
                                'a': 0.5
                            }
                        }
                    ]
                },
                "id": "area_2"
            },
            {
                "_area": {
                    "version": 1.0,
                    "id": "root",
                    "display_name": None,
                    "regions": []
                },
                "id": "root"
            }
        ],
        "adjacency": [
            [],
            [],
            [
                {
                    "id": "area_1"
                },
                {
                    "id": "area_2"
                }
            ]
        ]
    }

    return am, json_dict


class ReadWriteTest(unittest.TestCase):

    def test_read_write_area(self):
        am1, json_dict = make_tree()
        test_path = './test_area.json'
        try:
            am1.write_graph(test_path)

            with open(test_path, 'r') as fp:
                new_json_dict = json.load(fp)

            self.assertDictEqual(new_json_dict, json_dict)

            am2 = AreaManager.read_graph(test_path)

            self.assertEqual(am1, am2)
        finally:
            try:
                os.remove(test_path)  # Delete the test file after
            except Exception:
                pass  # Don't care if this fails


if __name__ == '__main__':
    unittest.main()
