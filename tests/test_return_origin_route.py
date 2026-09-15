# coding=utf-8
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

    def test_idle_position_far_from_route_is_rejected(self):
        with self.assertRaises(ReturnOriginRouteError) as caught:
            build_return_to_origin_tasks(self.config, 32.001, 118.001)
        self.assertEqual('RETURN_POSITION_OFF_ROUTE', caught.exception.code)


if __name__ == '__main__':
    unittest.main()

