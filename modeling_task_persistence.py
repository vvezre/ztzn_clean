# coding=utf-8
import copy
import re


try:
    text_type = unicode
except NameError:
    text_type = str


INVALID_TASK_NAME_CHARS = re.compile(r'[<>:"/\\|?*\x00-\x1f]')
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
