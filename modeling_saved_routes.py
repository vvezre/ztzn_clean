# coding=utf-8

"""读取 FSM 已命名保存的任务，并组装云平台/前端查询所需的完整路线列表。

一条“已保存路线”由两部分数据组成：
    1. 任务 JSON：任务名称、modelId、机器人实际执行的 taskList；
    2. 建模草稿：modelId 对应的区域点、连接点和原始建模信息。

本模块把两部分合并为 routes，每条 route 同时包含 areaPoints、linkPoints、pathPoints。
它只读取磁盘中已经保存的数据，不依赖内存中的当前建模会话，因此用于“按设备查询所有
已保存路线”。
"""

from modeling_frontend import (
    frontend_area_points,
    frontend_link_points,
    frontend_path_points,
)
from modeling_task_persistence import ModelingTaskPersistenceError, normalize_task_name


def _safe_task_name(value):
    """复用保存阶段的任务名规则；无效名称返回 None，而不是让一次查询整体失败。"""
    try:
        return normalize_task_name(value)
    except ModelingTaskPersistenceError:
        return None


def _saved_area_order(task_config, model):
    """Return the cleaning-area order stored for one named route.

    New task files persist ``areaOrder`` directly because several saved routes
    may originate from the same model but use different orders.  Older task
    files do not contain this field, so they fall back to the task plan kept in
    their associated model.  Invalid or unavailable legacy data becomes an
    empty list instead of breaking the whole saved-route query.
    """
    area_order = (task_config or {}).get('areaOrder')
    if isinstance(area_order, (list, tuple)) and area_order:
        return list(area_order)

    task_plan = (model or {}).get('taskPlan')
    if isinstance(task_plan, dict):
        area_order = task_plan.get('areaOrder')
        if isinstance(area_order, (list, tuple)):
            return list(area_order)
    return []


def discover_saved_task_names(file_names, load_task_config):
    """从任务目录中找出真正完整、可加载的已保存任务。

    不能看到一个 ``*.json`` 就当作有效路线：文件名、JSON 内 taskName、modelId 和非空
    taskList 必须同时有效。这样备份文件、损坏文件和半写入文件不会出现在前端路线列表。

    ``load_task_config`` 由调用方注入，便于复用现有配置读取逻辑并进行单元测试。
    """
    names = []
    for file_name in file_names or []:
        if not file_name or not file_name.lower().endswith('.json'):
            continue
        # 文件名去掉 .json 后就是候选任务名，并按保存时相同规则校验。
        file_task_name = _safe_task_name(file_name[:-5])
        if not file_task_name:
            continue
        try:
            task_config = load_task_config(file_name)
        except Exception:
            continue
        if not isinstance(task_config, dict):
            continue
        # 文件名必须与 JSON 内 taskName 一致，防止把改名或残留文件当作另一条路线。
        configured_name = _safe_task_name(task_config.get('taskName'))
        model_id = task_config.get('modelId')
        task_list = task_config.get('taskList')
        if (
            configured_name != file_task_name
            or not model_id
            or not isinstance(task_list, list)
            or not task_list
        ):
            continue
        names.append(configured_name)
    return sorted(set(names))


def build_saved_routes(
        task_names, current_task_name, load_task_config, load_model,
        current_return_to_origin=True):
    """构建 ``get_saved_routes`` 命令最终返回的业务数据。

    参数：
        task_names：已发现的有效任务名；
        current_task_name：机器人当前真正选中的任务，而不是前端页面高亮项；
        load_task_config：读取任务 JSON 的函数；
        load_model：根据 modelId 读取建模草稿的函数。

    返回：
        ``routes`` 为全部已保存路线；``currentTaskName`` 告诉前端当前已选路线。
        每条路线中的 taskCount 是实际执行线段数，不等于清扫线数量，因为连接段、转场段
        和回程段也属于 taskList。
    """
    # 当前任务名也必须经过同一套安全校验；无效值不会误标记任一路线为 current。
    current_task_name = _safe_task_name(current_task_name)
    normalized_names = sorted(set(
        task_name
        for task_name in (_safe_task_name(value) for value in (task_names or []))
        if task_name
    ))

    routes = []
    for task_name in normalized_names:
        # 单个任务文件异常时返回空的对应数据，不阻断其他正常路线的查询。
        try:
            task_config = load_task_config(task_name)
        except Exception:
            task_config = {}
        if not isinstance(task_config, dict):
            task_config = {}

        # modelId 把“执行任务文件”与“建模点位草稿”关联起来。
        model_id = task_config.get('modelId')
        model = {}
        if model_id:
            try:
                model = load_model(model_id)
            except Exception:
                model = {}
        if not isinstance(model, dict):
            model = {}

        is_current = task_name == current_task_name
        selected_return_to_origin = (
            bool(current_return_to_origin) if is_current else True
        )
        variants = task_config.get('routeVariants') or {}
        variant_key = 'return' if selected_return_to_origin else 'noReturn'
        selected_variant = variants.get(variant_key) or {}

        # 当前路线返回它真正选中的版本；其他路线默认展示返回原点版本。
        task_list = task_config.get('taskList')
        if isinstance(selected_variant.get('tasks'), list):
            task_list = selected_variant.get('tasks')
        if not isinstance(task_list, list):
            task_list = []

        # 三类点位都在这里转换成前端约定的扁平结构，避免前端理解 FSM 内部分组格式。
        routes.append({
            'taskName': task_name,
            'modelId': model_id,
            'current': is_current,
            'returnToOrigin': selected_return_to_origin,
            'taskCount': len(task_list),
            'areaOrder': _saved_area_order(task_config, model),
            'areaPoints': frontend_area_points(model),
            'linkPoints': frontend_link_points(model),
            'pathPoints': frontend_path_points({'tasks': task_list}),
        })

    return {
        'routes': routes,
        'currentTaskName': current_task_name,
        'currentReturnToOrigin': bool(current_return_to_origin),
    }
