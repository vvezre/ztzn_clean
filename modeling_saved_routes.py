# coding=utf-8

from modeling_frontend import (
    frontend_area_points,
    frontend_link_points,
    frontend_path_points,
)
from modeling_task_persistence import ModelingTaskPersistenceError, normalize_task_name


def _safe_task_name(value):
    try:
        return normalize_task_name(value)
    except ModelingTaskPersistenceError:
        return None


def build_saved_routes(task_names, current_task_name, load_task_config, load_model):
    current_task_name = _safe_task_name(current_task_name)
    normalized_names = sorted(set(
        task_name
        for task_name in (_safe_task_name(value) for value in (task_names or []))
        if task_name
    ))

    routes = []
    for task_name in normalized_names:
        try:
            task_config = load_task_config(task_name)
        except Exception:
            task_config = {}
        if not isinstance(task_config, dict):
            task_config = {}

        model_id = task_config.get('modelId')
        model = {}
        if model_id:
            try:
                model = load_model(model_id)
            except Exception:
                model = {}
        if not isinstance(model, dict):
            model = {}

        task_list = task_config.get('taskList')
        if not isinstance(task_list, list):
            task_list = []

        routes.append({
            'taskName': task_name,
            'modelId': model_id,
            'current': task_name == current_task_name,
            'taskCount': len(task_list),
            'areaPoints': frontend_area_points(model),
            'linkPoints': frontend_link_points(model),
            'pathPoints': frontend_path_points({'tasks': task_list}),
        })

    return {
        'routes': routes,
        'currentTaskName': current_task_name,
    }
