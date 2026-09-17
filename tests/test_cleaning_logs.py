# coding=utf-8
import os
import shutil
import tempfile
import unittest

from cleaning_logs import CleaningLogStore


class CleaningLogStoreTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='cleaning-logs-')
        self.clock = [1789518731.0]
        self.store = CleaningLogStore(self.root, now=lambda: self.clock[0])
        self.identity = {'productId': '250006', 'serialNumber': '-T01250006'}
        self.route = {
            'areaPoints': [{'x': 0, 'y': 0}],
            'linkPoints': [{'x': 5, 'y': 5}],
            'pathPoints': [{'x': 0, 'y': 0}, {'x': 100, 'y': 0}],
        }

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def test_completed_run_is_listed_and_detail_keeps_snapshot(self):
        record = self.store.begin('clean_run_1', self.identity, u'测试路线', self.route)
        self.route['pathPoints'][0]['x'] = 999
        self.store.observe(0, 0, 90, at=self.clock[0])
        self.clock[0] += 1.1
        self.store.observe(100, 0, 92, at=self.clock[0])
        self.clock[0] += 3.0
        archived = self.store.finalize('COMPLETED', u'路线正常完成')

        self.assertEqual('COMPLETED', archived['status'])
        self.assertIsNone(archived['distanceMeters'])
        page = self.store.list_logs('250006', 1, 20)
        self.assertEqual(1, page['total'])
        self.assertNotIn('trajectory', page['list'][0])
        detail = self.store.get_log(record['id'], '250006')
        self.assertEqual(0, detail['plannedRoute']['pathPoints'][0]['x'])
        self.assertEqual([90.0, 92.0], [point['heading'] for point in detail['trajectory']['points']])
        self.assertTrue(all(isinstance(point['timestamp'], int) for point in detail['trajectory']['points']))

    def test_product_filter_and_pagination(self):
        for index in range(3):
            self.store.begin('run_{}'.format(index), self.identity, 'task{}'.format(index), self.route)
            self.clock[0] += 1
            self.store.finalize('STOPPED', u'用户手动停止')
            self.clock[0] += 1
        first = self.store.list_logs('250006', 1, 2)
        second = self.store.list_logs('250006', 2, 2)
        self.assertEqual(3, first['total'])
        self.assertEqual(2, len(first['list']))
        self.assertEqual(1, len(second['list']))
        self.assertEqual(0, self.store.list_logs('999999', 1, 20)['total'])
        self.assertIsNone(self.store.get_log(first['list'][0]['id'], '999999'))

    def test_interrupted_running_record_is_failed_after_restart(self):
        record = self.store.begin('interrupted', self.identity, 'task', self.route)
        self.clock[0] += 5
        restored = CleaningLogStore(self.root, now=lambda: self.clock[0])
        detail = restored.get_log(record['id'], '250006')
        self.assertEqual('FAILED', detail['status'])
        self.assertEqual(u'上位机重启，任务异常中断', detail['endReason'])

    def test_same_run_begin_is_idempotent(self):
        first = self.store.begin('same', self.identity, 'task', self.route)
        second = self.store.begin('same', self.identity, 'task', {'pathPoints': []})
        self.assertEqual(first['id'], second['id'])
        self.assertEqual(1, self.store.list_logs('250006', 1, 20)['total'])


if __name__ == '__main__':
    unittest.main()
