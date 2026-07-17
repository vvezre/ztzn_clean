ACTIVE_TASK_ACTIONS = {
    'auto_drive',
    'go_on',
    'return_to_point',
    'go_to_point',
    'multi_go_to_point',
    'modeling_task',
}

ACTIVE_TASK_STATES = {'READY', 'RUNNING', 'PAUSED', 'STOPPING'}


def is_runtime_task_visible(action=None, control_state=None):
    action_value = str(action or '').strip()
    state_value = str(control_state or '').strip().upper()
    return action_value in ACTIVE_TASK_ACTIONS and state_value in ACTIVE_TASK_STATES


def live_value_from_report(value, report_at=None):
    if report_at is None:
        return None
    return value


def live_heading_from_location(lat=None, lon=None, heading=None):
    if lat is None or lon is None:
        return None
    return heading


def visible_task_field(value, action=None, control_state=None):
    if not is_runtime_task_visible(action, control_state):
        return None
    return value
