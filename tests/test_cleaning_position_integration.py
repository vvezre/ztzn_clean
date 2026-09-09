# coding=utf-8
"""Execute just telemetry adapters from main.py, never import hardware main."""
import ast
import copy
import io
import os
import unittest
from types import SimpleNamespace

from cleaning_position import CleaningPositionHistory, CleaningPositionService
from test_cleaning_position import task, gps


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with io.open(os.path.join(ROOT, 'main.py'), 'r', encoding='utf-8') as handle:
    SOURCE = handle.read()
TREE = ast.parse(SOURCE)


def load_functions(names, namespace):
    nodes = [copy.deepcopy(node) for node in TREE.body
             if isinstance(node, ast.FunctionDef) and node.name in names]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), 'main.py', 'exec'), namespace)
    return namespace


class CleaningMainIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.logs = []
        self.service = CleaningPositionService(CleaningPositionHistory())
        self.namespace = {
            'cleaning_position_service': self.service,
            'active_runtime_task_token': 'runtime-1',
            'global_cur_rtk_lat': gps(0)[0], 'global_cur_rtk_lon': gps(0)[1],
            '_build_rtk_runtime_detail': lambda: {'rtkFixAvailable': True},
            'logger': SimpleNamespace(warning=self.logs.append),
        }
        load_functions(('_cleaning_position_sample', '_begin_cleaning_position_run',
                        '_notify_cleaning_position_state',
                        '_notify_cleaning_position_start_failed'), self.namespace)

    def test_production_begin_and_state_notifications_are_worker_only(self):
        config = task()
        before = copy.deepcopy(config)
        self.namespace['_begin_cleaning_position_run'](config, 'runtime-1')
        self.assertIsNone(self.service.realtime()['runId'])
        self.assertEqual(config, before)
        self.namespace['_notify_cleaning_position_state']('RUNNING')
        self.service.poll(gps(0) + (True,), 'RUNNING', 'runtime-1')
        self.assertEqual(0, self.service.realtime()['x'])
        self.namespace['global_cur_rtk_lon'] = gps(15)[1]
        self.namespace['active_runtime_task_token'] = ''
        self.namespace['_notify_cleaning_position_state']('STOPPED')
        self.service.poll(gps(900) + (True,), 'STOPPED', '')
        self.assertEqual({'x': 15, 'y': 0}, self.service.history()['points'][-1])

    def test_missing_service_during_startup_is_harmless(self):
        self.namespace.pop('cleaning_position_service')
        self.namespace['_notify_cleaning_position_state']('INITIALIZING')
        self.namespace['_begin_cleaning_position_run'](task(), 'runtime-1')
        self.assertEqual([], self.logs)

    def test_telemetry_exception_cannot_propagate_to_control_thread(self):
        def broken(*args, **kwargs):
            raise RuntimeError('test queue error')
        self.namespace['cleaning_position_service'] = SimpleNamespace(
            begin=broken, state=broken, start_failed=broken)
        self.namespace['_begin_cleaning_position_run'](task(), 'runtime-1')
        self.namespace['_notify_cleaning_position_state']('RUNNING')
        self.namespace['_notify_cleaning_position_start_failed']()
        self.assertEqual(3, len(self.logs))

    def test_production_start_failure_notification_is_worker_only(self):
        self.namespace['_notify_cleaning_position_start_failed']()
        self.assertEqual('IDLE', self.service.realtime()['controlState'])
        self.service.poll(gps(0) + (True,), 'BLOCKED', '')
        self.assertEqual('START_FAILED', self.service.realtime()['controlState'])

    def test_begin_hook_is_after_validation_and_before_segment_execution(self):
        node = next(node for node in TREE.body if isinstance(node, ast.FunctionDef) and node.name == 'autoDriveByRTKThread')
        source = ast.get_source_segment(SOURCE, node)
        self.assertLess(source.index("'TASK_PATH_EMPTY'"), source.index('_begin_cleaning_position_run'))
        self.assertLess(source.index('_begin_cleaning_position_run'), source.index('_run_task_segment_by_point_navigation'))

    def test_read_routes_dont_dispatch_motion_or_reset_modeling(self):
        with io.open(os.path.join(ROOT, 'lan_cloud_compat.py'), 'r', encoding='utf-8') as handle:
            tree = ast.parse(handle.read())
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name.startswith('lan_cloud_cleaning_'):
                names = [child.id for child in ast.walk(node) if isinstance(child, ast.Name)]
                self.assertNotIn('run_command', names)
                self.assertNotIn('terminal_data', names)
