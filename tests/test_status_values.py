import unittest

from status_values import (
    is_runtime_task_visible,
    live_heading_from_location,
    live_value_from_report,
    visible_task_field,
)


class StatusValuesTest(unittest.TestCase):
    def test_live_hardware_value_is_none_without_a_report(self):
        self.assertIsNone(live_value_from_report(350, report_at=None))
        self.assertEqual(live_value_from_report(0, report_at=123), 0)
        self.assertEqual(live_value_from_report(30, report_at=123), 30)

    def test_heading_is_none_until_location_is_available(self):
        self.assertIsNone(live_heading_from_location(None, None, 0.0))
        self.assertIsNone(live_heading_from_location(32.0, None, 90.0))
        self.assertEqual(live_heading_from_location(32.0, 118.0, 90.0), 90.0)

    def test_task_fields_are_hidden_when_robot_is_not_running_a_task(self):
        self.assertFalse(is_runtime_task_visible('parking', 'STOPPED'))
        self.assertIsNone(visible_task_field('002', action='parking', control_state='STOPPED'))
        self.assertIsNone(visible_task_field(16, action='idle', control_state='COMPLETE'))

    def test_task_fields_are_visible_for_runtime_task_actions(self):
        self.assertTrue(is_runtime_task_visible('auto_drive', 'RUNNING'))
        self.assertEqual(visible_task_field('002', action='auto_drive', control_state='RUNNING'), '002')
        self.assertEqual(visible_task_field(16, action='multi_go_to_point', control_state='PAUSED'), 16)


if __name__ == "__main__":
    unittest.main()
