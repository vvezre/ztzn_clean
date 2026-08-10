# coding=utf-8
import copy
import re


try:
    text_type = unicode
except NameError:
    text_type = str


# Keep the regular expression itself as Unicode.  On Python 2.7 a byte-pattern
# character class containing ``\x00-\x1f`` can incorrectly match the first
# character of a non-ASCII Unicode task name, causing valid Chinese route names
# to be rejected as unsafe.  Python 3 does not expose that behaviour.
INVALID_TASK_NAME_CHARS = re.compile(u'[<>:"/\\|?*\x00-\x1f]')
WINDOWS_RESERVED_NAMES = set(
    ["CON", "PRN", "AUX", "NUL"]
    + ["COM{}".format(index) for index in range(1, 10)]
    + ["LPT{}".format(index) for index in range(1, 10)]
)


class ModelingTaskPersistenceError(Exception):
    def __init__(self, code, message):
        super(ModelingTaskPersistenceError, self).__init__(message)
        self.code = code
        self.message = message


def normalize_task_name(task_name):
    """
    校验并标准化用户输入的路线名称。

    路线名后续会作为任务文件名，因此需要同时满足业务和文件系统安全要求：
    - 不能为空，自动去除首尾空格。
    - 最长 64 个字符。
    - 不允许 < > : " / \\ | ? * 和控制字符。
    - 不允许 .、.. 、以点结尾和 Windows 保留名 CON/PRN/AUX/NUL/COMx/LPTx。

    这些限制也防止通过 taskName 构造非法路径。
    """
    if task_name is None:
        value = ""
    elif isinstance(task_name, bytes):
        task_name = task_name.decode("utf-8")
        value = text_type(task_name).strip()
    else:
        value = text_type(task_name).strip()
    if not value:
        raise ModelingTaskPersistenceError(
            "TASK_NAME_REQUIRED",
            "taskName is required",
        )
    if len(value) > 64:
        raise ModelingTaskPersistenceError(
            "TASK_NAME_TOO_LONG",
            "taskName must not exceed 64 characters",
        )
    if value in (".", "..") or value.endswith(".") or INVALID_TASK_NAME_CHARS.search(value):
        raise ModelingTaskPersistenceError(
            "TASK_NAME_INVALID",
            "taskName contains invalid characters",
        )
    stem = value.split(".", 1)[0].upper()
    if stem in WINDOWS_RESERVED_NAMES:
        raise ModelingTaskPersistenceError(
            "TASK_NAME_INVALID",
            "taskName is reserved by the operating system",
        )
    return value


def build_named_task(base_config, current_path, task_name):
    """
    将 finish_modeling 生成的当前路径固化为一条可命名、可查询、可选择的任务。

    输入：
    - base_config：小车原有运行参数，例如速度、充电桩和车库参数。
    - current_path：finish_modeling 生成的当前建模结果。
    - task_name：用户输入的路线名称。

    输出是一份完整任务配置：
    - 原有运行参数继续保留。
    - taskName 用于列表展示和选择。
    - modelId 关联到产生它的建模记录。
    - taskList 是机器人真正执行的唯一有序路径段列表。
    - startLat/startLon 是路线起点。
    - originHeading/heading 为兼容旧任务文件继续保留；新版执行时会用实时位置重新算方向。

    保存的 taskList 与规划结果保持一致，不在保存环节重新生成路线。
    """
    task_name = normalize_task_name(task_name)
    if not isinstance(current_path, dict):
        raise ModelingTaskPersistenceError(
            "MODELING_PATH_NOT_READY",
            "modeling path is not ready",
        )
    task_plan = current_path.get("taskPlan")
    if not isinstance(task_plan, dict) or task_plan.get("status") != "ready":
        raise ModelingTaskPersistenceError(
            "MODELING_PATH_NOT_READY",
            "modeling path is not ready",
        )
    tasks = task_plan.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ModelingTaskPersistenceError(
            "MODELING_PATH_EMPTY",
            "modeling path contains no executable tasks",
        )

    task_config = copy.deepcopy(base_config if isinstance(base_config, dict) else {})
    task_config["taskName"] = task_name
    task_config["modelId"] = current_path.get("modelId")
    task_config["taskList"] = copy.deepcopy(tasks)

    # startLat/startLon 用于启动前检查是否位于路线原点。
    # heading 字段仅为兼容旧任务和前端展示保留，真车执行不再直接采用该预存航向。
    first_task = tasks[0]
    if isinstance(first_task, dict):
        if first_task.get("startLat") is not None:
            task_config["startLat"] = first_task.get("startLat")
        if first_task.get("startLon") is not None:
            task_config["startLon"] = first_task.get("startLon")
        if first_task.get("heading") is not None:
            task_config["originHeading"] = first_task.get("heading")
            task_config["heading"] = first_task.get("heading")

    return task_config


def is_same_named_task(task_config, current_path, task_name):
    """
    验证保存后的任务是否与当前建模结果完全一致。

    同时比较路线名、modelId 和整个 taskList，用于保存后校验，
    避免只写入了名称但路径内容丢失或与当前规划不一致。
    """
    if not isinstance(task_config, dict) or not isinstance(current_path, dict):
        return False
    try:
        expected_name = normalize_task_name(task_name)
        configured_name = normalize_task_name(task_config.get("taskName"))
    except ModelingTaskPersistenceError:
        return False
    task_list = task_config.get("taskList")
    current_task_plan = current_path.get("taskPlan")
    current_tasks = (
        current_task_plan.get("tasks")
        if isinstance(current_task_plan, dict)
        else None
    )
    return (
        configured_name == expected_name
        and task_config.get("modelId") == current_path.get("modelId")
        and isinstance(task_list, list)
        and bool(task_list)
        and isinstance(current_tasks, list)
        and task_list == current_tasks
    )
