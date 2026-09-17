# coding=utf-8
import math
import unittest

from return_origin_route import ReturnOriginRouteError, build_return_to_origin_tasks


def segment(task_id, start_x, end_x, start_lon, end_lon):
    return {
        'id': task_id,
        'mode': 2,
        'areaNumber': 1,
        'startX': start_x,
        'startY': 0,
        'endX': end_x,
        'endY': 0,
        'startLat': 32.0,
        'startLon': start_lon,
        'endLat': 32.0,
        'endLon': end_lon,
        'heading': 180,
        'length': abs(end_x - start_x),
    }


def coordinate(x, y):
    latitude = 32.0 + (float(y) / 100.0) / 111194.9266
    longitude = 118.0 + (float(x) / 100.0) / (
        111194.9266 * math.cos(math.radians(32.0))
    )
    return latitude, longitude


def segment_xy(task_id, start_x, start_y, end_x, end_y):
    start_lat, start_lon = coordinate(start_x, start_y)
    end_lat, end_lon = coordinate(end_x, end_y)
    return {
        'id': task_id,
        'mode': 2,
        'areaNumber': 1,
        'startX': start_x,
        'startY': start_y,
        'endX': end_x,
        'endY': end_y,
        'startLat': start_lat,
        'startLon': start_lon,
        'endLat': end_lat,
        'endLon': end_lon,
        'heading': 0,
        'length': int(round(math.hypot(end_x - start_x, end_y - start_y))),
    }


def model_polygon(points):
    return {
        'groups': [{
            'id': 'area-1',
            'areaNumber': 1,
            'points': [
                {'x': x, 'y': y}
                for x, y in points
            ],
        }],
    }


def model_areas(areas):
    return {
        'groups': [
            {
                'id': 'area-{}'.format(index + 1),
                'areaNumber': index + 1,
                'points': [{'x': x, 'y': y} for x, y in points],
            }
            for index, points in enumerate(areas)
        ],
    }


class ReturnOriginRouteTests(unittest.TestCase):
    def setUp(self):
        # About one metre of longitude at latitude 32 degrees.
        self.lon_step = 0.0000106
        tasks = [
            segment(1, 0, 100, 118.0, 118.0 + self.lon_step),
            segment(2, 100, 200, 118.0 + self.lon_step, 118.0 + self.lon_step * 2),
            segment(3, 200, 300, 118.0 + self.lon_step * 2, 118.0 + self.lon_step * 3),
        ]
        self.config = {
            'taskName': u'返航测试',
            'startLat': 32.0,
            'startLon': 118.0,
            'taskList': tasks,
            'routeVariants': {
                'return': {'tasks': tasks},
                'noReturn': {'tasks': tasks[:-1]},
            },
        }

    def test_completed_no_return_route_uses_saved_segments_back_to_origin(self):
        result = build_return_to_origin_tasks(
            self.config,
            32.0,
            118.0 + self.lon_step * 2,
        )
        self.assertEqual(2, len(result))
        self.assertEqual([(200, 100), (100, 0)], [
            (item['startX'], item['endX']) for item in result
        ])
        self.assertTrue(all(item['mode'] == 2 for item in result))

    def test_active_segment_anchors_mid_segment_without_direct_origin_shortcut(self):
        active = self.config['taskList'][1]
        result = build_return_to_origin_tasks(
            self.config,
            32.0,
            118.0 + self.lon_step * 1.5,
            active_segment=active,
        )
        self.assertEqual(2, len(result))
        self.assertAlmostEqual(118.0 + self.lon_step, result[0]['endLon'], places=8)
        self.assertAlmostEqual(118.0, result[-1]['endLon'], places=8)

    def test_already_at_origin_has_no_motion_task(self):
        self.assertEqual([], build_return_to_origin_tasks(self.config, 32.0, 118.0))

    def test_idle_position_far_from_route_joins_nearest_segment_without_node_limit(self):
        result = build_return_to_origin_tasks(self.config, 32.001, 118.001)

        self.assertGreater(len(result), 1)
        self.assertEqual((0, 0), (result[-1]['endX'], result[-1]['endY']))

    def test_idle_vehicle_in_middle_of_long_segment_is_not_rejected(self):
        long_segment = segment_xy(1, 0, 0, 300, 0)
        config = {
            'taskName': u'长线段中点返航',
            'startLat': long_segment['startLat'],
            'startLon': long_segment['startLon'],
            'taskList': [long_segment],
        }
        current_lat, current_lon = coordinate(150, 0)

        result = build_return_to_origin_tasks(config, current_lat, current_lon)

        self.assertEqual(1, len(result))
        self.assertEqual((150, 0, 0, 0), (
            result[0]['startX'], result[0]['startY'],
            result[0]['endX'], result[0]['endY'],
        ))

    def test_idle_vehicle_joins_projection_before_following_route_home(self):
        long_segment = segment_xy(1, 0, 0, 300, 0)
        config = {
            'taskName': u'投影点接入',
            'startLat': long_segment['startLat'],
            'startLon': long_segment['startLon'],
            'taskList': [long_segment],
        }
        current_lat, current_lon = coordinate(150, 50)

        result = build_return_to_origin_tasks(config, current_lat, current_lon)

        self.assertEqual(2, len(result))
        self.assertEqual((150, 50, 150, 0), (
            result[0]['startX'], result[0]['startY'],
            result[0]['endX'], result[0]['endY'],
        ))
        self.assertEqual((150, 0, 0, 0), (
            result[1]['startX'], result[1]['startY'],
            result[1]['endX'], result[1]['endY'],
        ))

    def test_variant_midpoint_splits_long_edge_instead_of_forcing_reversal(self):
        full = segment_xy(1, 0, 0, 0, 100)
        upper_half = segment_xy(2, 0, 50, 0, 100)
        config = {
            'taskName': u'中点拆线',
            'startLat': full['startLat'],
            'startLon': full['startLon'],
            'taskList': [full],
            'routeVariants': {
                'return': {'tasks': [full]},
                'noReturn': {'tasks': [upper_half]},
            },
        }
        current_lat, current_lon = coordinate(0, 50)

        result = build_return_to_origin_tasks(config, current_lat, current_lon)

        self.assertEqual(1, len(result))
        self.assertEqual((0, 50, 0, 0), (
            result[0]['startX'], result[0]['startY'],
            result[0]['endX'], result[0]['endY'],
        ))

    def test_idle_vehicle_returns_directly_when_whole_line_is_inside_area(self):
        left = segment_xy(1, 0, 0, 0, 100)
        top = segment_xy(2, 0, 100, 200, 100)
        config = {
            'taskName': u'区域内直返',
            'startLat': left['startLat'],
            'startLon': left['startLon'],
            'taskList': [left, top],
        }
        model = model_polygon([(0, 0), (0, 100), (200, 100), (200, 0)])
        current_lat, current_lon = coordinate(200, 100)

        result = build_return_to_origin_tasks(
            config,
            current_lat,
            current_lon,
            model=model,
        )

        self.assertEqual(1, len(result))
        self.assertEqual((200, 100, 0, 0), (
            result[0]['startX'], result[0]['startY'],
            result[0]['endX'], result[0]['endY'],
        ))

    def test_active_cleaning_keeps_saved_path_even_inside_same_area(self):
        left = segment_xy(1, 0, 0, 0, 100)
        top = segment_xy(2, 0, 100, 200, 100)
        config = {
            'taskName': u'清扫中返航',
            'startLat': left['startLat'],
            'startLon': left['startLon'],
            'taskList': [left, top],
        }
        model = model_polygon([(0, 0), (0, 100), (200, 100), (200, 0)])
        current_lat, current_lon = coordinate(200, 100)

        result = build_return_to_origin_tasks(
            config,
            current_lat,
            current_lon,
            active_segment=top,
            model=model,
        )

        self.assertEqual(2, len(result))
        self.assertEqual((0, 100), (result[0]['endX'], result[0]['endY']))
        self.assertEqual((0, 0), (result[-1]['endX'], result[-1]['endY']))

    def test_direct_shortcut_is_rejected_when_it_crosses_concave_gap(self):
        polygon = [
            (0, 0), (100, 0), (100, 100), (70, 100),
            (70, 30), (30, 30), (30, 100), (0, 100),
        ]
        tasks = [
            segment_xy(index + 1, start[0], start[1], end[0], end[1])
            for index, (start, end) in enumerate(zip(polygon, polygon[1:]))
        ]
        config = {
            'taskName': u'凹形区域',
            'startLat': tasks[0]['startLat'],
            'startLon': tasks[0]['startLon'],
            'taskList': tasks,
        }
        current_lat, current_lon = coordinate(100, 100)

        result = build_return_to_origin_tasks(
            config,
            current_lat,
            current_lon,
            model=model_polygon(polygon),
        )

        self.assertGreater(len(result), 1)
        self.assertNotEqual((0, 0), (result[0]['endX'], result[0]['endY']))

    def test_direct_shortcut_is_rejected_between_separate_areas(self):
        tasks = [
            segment_xy(1, 0, 0, 0, 50),
            segment_xy(2, 0, 50, 100, 50),
            segment_xy(3, 100, 50, 300, 50),
            segment_xy(4, 300, 50, 400, 50),
        ]
        config = {
            'taskName': u'跨区域返航',
            'startLat': tasks[0]['startLat'],
            'startLon': tasks[0]['startLon'],
            'taskList': tasks,
        }
        model = model_areas([
            [(0, 0), (0, 100), (100, 100), (100, 0)],
            [(300, 0), (300, 100), (400, 100), (400, 0)],
        ])
        current_lat, current_lon = coordinate(400, 50)

        result = build_return_to_origin_tasks(
            config,
            current_lat,
            current_lon,
            model=model,
        )

        self.assertGreater(len(result), 1)
        self.assertEqual((300, 50), (result[0]['endX'], result[0]['endY']))
        self.assertEqual((0, 0), (result[-1]['endX'], result[-1]['endY']))


if __name__ == '__main__':
    unittest.main()
