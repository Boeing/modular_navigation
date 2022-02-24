import unittest

from graph_map.util import to_simple
from graph_map.node import Node
from graph_map.area import Area, Region, Color


class TrialClass:
    def __init__(self, a, b):
        self.a = a
        self.b = b

    def to_simple_dict(self):
        return {'TC_a': to_simple(self.a), 'TC_b': to_simple(self.b)}


class SimplifyDataTest(unittest.TestCase):

    def test_simple_list(self):
        ls = list(range(10))
        out = to_simple(ls)
        self.assertListEqual(out, ls)

    def test_simple_set(self):
        ss = set(range(10))
        out = to_simple(ss)
        self.assertListEqual(out, list(ss))

    def test_simple_dict(self):
        ds = dict(zip('abcde', range(5)))
        out = to_simple(ds)
        self.assertDictEqual(ds, out)

    def test_simple_tuple(self):
        ts = (1, 2, 3, 4)
        out = to_simple(ts)
        self.assertListEqual(out, list(ts))

    def test_str(self):
        strs = 'apple'
        out = to_simple(strs)
        self.assertEqual(strs, out)

    def test_custom_class(self):
        tc = TrialClass([1, 3], TrialClass(1, 2))
        out = to_simple(tc)
        expected = {'TC_a': [1, 3], 'TC_b': {'TC_a': 1, 'TC_b': 2}}
        self.assertDictEqual(out, expected)

    def test_simplify_node(self):
        node_id = 'William_Ko'
        node_display_name = 'Will Ko'
        x = 23
        y = 46

        node = Node(id=node_id, x=x, y=y, display_name=node_display_name)
        out = to_simple(node)

        # Not all fields have been implemented
        expected = {Node.__name__: {'display_name': node_display_name,
                                    'id': 'William_Ko',
                                    'version': Node.version,
                                    'x': x,  # x, y might change into a pose like object
                                    'y': y,
                                    'theta': 0.0,
                                    'parent': None,
                                    'children': {}
                                    }}
        self.assertDictEqual(out, expected)

    def test_simplify_area(self):
        area_id = 'Leng Vongchanh'
        area_display_name = 'Ben Vongchanh'
        points = [[0, 0], [0, 1], [1, 1], [1, 0]]
        _color = {'r': 0.5, 'b': 0.5, 'g': 0.5, 'a': 0.5}
        color = Color(**_color)
        regions = [Region(points, color)]

        area = Area(id=area_id, display_name=area_display_name, regions=regions)
        out = to_simple(area)

        # Not all fields have been implemented
        expected = {'display_name': area_display_name,
                    'id': area_id,
                    'version': Area.version,
                    'regions': [{'points': points+[points[0]], 'color': _color}]
                    }
        self.assertDictEqual(out, expected)

        obj = Area.from_dict(out)
        self.assertEqual(area.id, obj.id)
        self.assertEqual(area.display_name, obj.display_name)
        self.assertEqual(area.regions, obj.regions)


if __name__ == '__main__':
    unittest.main()
