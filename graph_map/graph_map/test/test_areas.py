import unittest

from graph_map.util import to_simple
from graph_map.area import Area, Region, PoseLike, Color


class AreaTest(unittest.TestCase):

    def test_region_is_in(self):
        area_id = 'Leng Vongchanh'
        area_display_name = 'Ben Vongchanh'
        region_1 = Region([[0, 0], [0, 1], [1, 1], [1, 0]])  # Unit square at (0,0)
        region_2 = Region([[2, 0], [2, 1], [3, 1], [3, 0]])  # Unit square at (2,0)
        regions = [region_1, region_2]

        pose_1 = PoseLike(0.5, 0.5)  # In region_1 but not in region_2
        pose_2 = PoseLike(2.5, 0.5)  # In region_2 but not in region_1

        self.assertEqual(regions[0].is_in(pose_1), True)
        self.assertEqual(regions[0].is_in(pose_2), False)

        self.assertEqual(regions[1].is_in(pose_1), False)
        self.assertEqual(regions[1].is_in(pose_2), True)

        area = Area(id=area_id, display_name=area_display_name, regions=regions)

        # Area has both regions, so both poses should be in the area
        self.assertEqual(area.is_in(pose_1), True)
        self.assertEqual(area.is_in(pose_2), True)

    def test_simplify_area(self):
        area_id = 'Leng Vongchanh'
        area_display_name = 'Ben Vongchanh'
        points = [[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]]
        _color = {'r': 1, 'b': 1, 'g': 1, 'a': 1}
        color = Color(**_color)
        regions = [Region(points, color)]

        area = Area(id=area_id, display_name=area_display_name, regions=regions)
        out = to_simple(area)

        # points needs to include the last point to close the polygon (shapely)
        expected = {'display_name': area_display_name,
                    'id': area_id,
                    'version': Area.version,
                    'regions': [{'color': _color, 'points': points+[points[0]]}]
                    }

        self.assertDictEqual(out, expected)

        obj = Area.from_dict(out)
        self.assertEqual(area.id, obj.id)
        self.assertEqual(area.display_name, obj.display_name)
        self.assertEqual(area.regions, obj.regions)


if __name__ == '__main__':
    unittest.main()
