# coding=utf-8
import time

from continuous_route import attach_continuations, collect_continuous_run


class ModelingExecutionError(Exception):
    pass


def _number(value):
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result


def _positive_int(value):
    try:
        result = int(value)
    except (TypeError, ValueError):
        return None
    return result if result > 0 else None


def _timestamp(now=None):
    if callable(now):
        return int(now())
    if now is not None:
        return int(now)
    return int(time.time())


def _require_number(task, field_name):
    value = _number(task.get(field_name))
    if value is None:
        raise ModelingExecutionError("task {} is required".format(field_name))
    return value


def _normalize_segment(task, index, speed):
    if not isinstance(task, dict):
        raise ModelingExecutionError("task segment must be object")

    mode = _positive_int(task.get("mode"))
    if mode is None:
        raise ModelingExecutionError("task mode is invalid")

    segment = dict(task)
    segment["id"] = int(task.get("id") or index + 1)
    segment["index"] = index
    segment["mode"] = mode
    segment["startLat"] = _require_number(task, "startLat")
    segment["startLon"] = _require_number(task, "startLon")
    segment["endLat"] = _require_number(task, "endLat")
    segment["endLon"] = _require_number(task, "endLon")
    segment["heading"] = _require_number(task, "heading") % 360.0
    segment["length"] = _require_number(task, "length")
    segment["speed"] = speed
    return segment


def build_execution_plan(model_id, task_plan, speed=None, now=None):
    if not model_id:
        raise ModelingExecutionError("model id is required")
    if not isinstance(task_plan, dict) or task_plan.get("status") != "ready":
        raise ModelingExecutionError("ready task plan is required")

    speed = _positive_int(speed)
    if speed is None:
        raise ModelingExecutionError("valid runtime speed is required")

    tasks = task_plan.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ModelingExecutionError("task plan has no executable segments")

    # 把保存任务中的每个task复制并标准化，统一补充index、运行速度和数值类型。
    # 列表顺序就是唯一执行顺序，运行时不会重新规划或跳过成功任务。
    segments = [
        _normalize_segment(task, index, speed)
        for index, task in enumerate(tasks)
    ]
    return {
        "status": "ready",
        "action": "modeling_task",
        "modelId": str(model_id),
        "createdAt": _timestamp(now),
        "speed": speed,
        "taskCount": len(segments),
        "summary": dict(task_plan.get("summary") or {}),
        "segments": segments,
    }


def _progress_payload(plan, status, current_index=0, segment=None, message=""):
    payload = {
        "status": status,
        "action": "modeling_task",
        "modelId": plan.get("modelId"),
        "currentIndex": int(current_index or 0),
        "total": int(plan.get("taskCount") or 0),
        "message": message,
    }
    if segment:
        payload["currentTaskId"] = segment.get("id")
        payload["mode"] = segment.get("mode")
    return payload


def execute_modeling_plan(plan, run_segment, update_progress=None, should_stop=None, now=None):
    if not isinstance(plan, dict) or plan.get("action") != "modeling_task":
        raise ModelingExecutionError("modeling execution plan is required")
    if not callable(run_segment):
        raise ModelingExecutionError("segment runner is required")

    update_progress = update_progress if callable(update_progress) else (lambda state: None)
    should_stop = should_stop if callable(should_stop) else (lambda: False)
    # 复制任务段列表，避免执行期间其他线程修改原计划对象。
    segments = list(plan.get("segments") or [])
    # completed只统计已经明确返回成功的任务段，用于前端进度和中断恢复诊断。
    completed = 0

    # 在执行第一段之前先发布running状态，total等于待执行总段数。
    update_progress(_progress_payload(plan, "running", 0, None, "modeling task started"))
    index = 0
    while index < len(segments):
        run = collect_continuous_run(segments, index)
        if not run:
            run = [segments[index]]
        segment = attach_continuations(run)
        run_count = len(run)
        # 每段开始前检查停止请求，确保用户点“停止”后不会继续启动下一段。
        if should_stop():
            result = _progress_payload(plan, "stopped", completed, segment, "modeling task stopped")
            result["code"] = "MODELING_TASK_STOPPED"
            result["completedCount"] = completed
            result["finishedAt"] = _timestamp(now)
            update_progress(result)
            return result

        # 连续折线只调用一次底层直行控制。其余子段作为运行时折线数据附加在首段，
        # 车辆通过投影和前视点连续跟踪整条折线；经过普通浮动点时不结束电机控制，
        # 也不会重新发送启动命令或重新按短距离目标计算航向。
        update_progress(_progress_payload(plan, "running", index, segment, "segment running"))
        try:
            # run_segment接入真车RTK、转向、电机、滚刷和刹车；返回True/1才算成功。
            ok = run_segment(segment)
        except Exception as exc:
            ok = False
            error_message = str(exc)
        else:
            error_message = ""
        if not ok:
            # 单段失败后不允许继续后续路线；根据是否收到人工停止区分stopped/blocked。
            if should_stop():
                result = _progress_payload(plan, "stopped", completed, segment, "modeling task stopped")
                result["code"] = "MODELING_TASK_STOPPED"
                result["completedCount"] = completed
                result["finishedAt"] = _timestamp(now)
                update_progress(result)
                return result
            result = _progress_payload(plan, "blocked", index, segment, error_message or "segment failed")
            result["code"] = "MODELING_SEGMENT_FAILED"
            result["completedCount"] = completed
            result["finishedAt"] = _timestamp(now)
            update_progress(result)
            return result

        # 整条连续折线成功后，按其中真实包含的任务段数量更新进度。
        completed += run_count
        update_progress(_progress_payload(plan, "running", completed, segment, "segment complete"))
        index += run_count

    # 所有任务段均成功后才发布complete。是否回到原点由所选路线版本决定：
    # returnToOrigin=true 的任务末段回原点，false 的任务在最后清扫区域结束。
    result = _progress_payload(plan, "complete", completed, None, "modeling task complete")
    result["code"] = "MODELING_TASK_COMPLETE"
    result["completedCount"] = completed
    result["finishedAt"] = _timestamp(now)
    update_progress(result)
    return result
