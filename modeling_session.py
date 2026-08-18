# coding=utf-8
import io
import json
import math
import os
import threading
import time

from modeling_capture import MixedCaptureError, resolve_mixed_capture
from modeling_coordinates import find_model_origin, lat_lon_to_model_xy_cm


SESSION_SCHEMA_VERSION = 1


class ModelingSessionError(Exception):
    def __init__(self, code, message):
        super(ModelingSessionError, self).__init__(message)
        self.code = code
        self.message = message


class ModelingSession(object):
    """Keeps the current mini-program modeling workflow on the robot."""

    def __init__(self, store, sample_point_provider, now=None):
        self.store = store
        self.sample_point_provider = sample_point_provider
        self._now = now or time.time
        self._lock = threading.RLock()
        self._state_path = os.path.join(self.store.root_dir, "active_session.json")

    def _timestamp(self):
        return int(self._now())

    def _read_state(self):
        if not os.path.exists(self._state_path):
            return None
        try:
            with io.open(self._state_path, "r", encoding="utf-8") as handle:
                state = json.load(handle)
        except (IOError, ValueError) as error:
            raise ModelingSessionError("MODELING_SESSION_INVALID", str(error))
        return state if isinstance(state, dict) else None

    def _write_state(self, state):
        state = dict(state)
        state["version"] = SESSION_SCHEMA_VERSION
        state["updatedAt"] = self._timestamp()
        temporary_path = self._state_path + ".tmp"
        serialized = json.dumps(state, ensure_ascii=True, indent=2, sort_keys=True)
        if not isinstance(serialized, type(u"")):
            serialized = serialized.decode("utf-8")
        with io.open(temporary_path, "w", encoding="utf-8") as handle:
            handle.write(serialized)
        getattr(os, "replace", os.rename)(temporary_path, self._state_path)
        return state

    def _require_state(self, allow_ready=False):
        state = self._read_state()
        if not state or not state.get("modelId"):
            raise ModelingSessionError("MODELING_SESSION_NOT_STARTED", "modeling has not been started")
        if not allow_ready and state.get("status") != "recording":
            raise ModelingSessionError("MODELING_SESSION_NOT_RECORDING", "modeling session is not recording")
        return state

    def _sample(self):
        if self.sample_point_provider is None:
            raise ModelingSessionError("RTK_SAMPLE_PROVIDER_MISSING", "sample provider is missing")
        return self.sample_point_provider()

    def _find_group(self, draft, group_id):
        return next((group for group in (draft.get("groups") or []) if group.get("id") == group_id), None)

    def _find_link(self, draft, link_id):
        return next((link for link in (draft.get("groupLinks") or []) if link.get("id") == link_id), None)

    def _finite_number(self, value):
        """把坐标字段转换为可参与距离计算的有限浮点数。"""
        if value is None or value == "":
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if not math.isnan(number) and not math.isinf(number) else None

    def _stored_point_xy(self, point):
        """读取已经统一到模型坐标系中的区域点坐标。"""
        x = self._finite_number((point or {}).get("x"))
        y = self._finite_number((point or {}).get("y"))
        return (x, y) if x is not None and y is not None else None

    def _sample_point_xy(self, draft, point):
        """把刚采样的连接点换算到当前模型的厘米坐标系。"""
        origin = find_model_origin(draft)
        xy = lat_lon_to_model_xy_cm(
            origin,
            (point or {}).get("lat"),
            (point or {}).get("lon"),
        )
        if xy is not None:
            return xy
        # 模拟器和部分测试会直接提供已经换算好的 x/y，RTK 不可用时保留兼容。
        return self._stored_point_xy(point)

    def _point_to_segment_distance(self, point, start, end):
        """计算一个点到有限边界线段的最短距离，单位为厘米。"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        projection = (
            (point[0] - start[0]) * dx
            + (point[1] - start[1]) * dy
        ) / float(length_squared)
        projection = max(0.0, min(1.0, projection))
        nearest = (
            start[0] + projection * dx,
            start[1] + projection * dy,
        )
        return math.hypot(point[0] - nearest[0], point[1] - nearest[1])

    def _group_boundary_distance(self, group, point_xy):
        """计算连接点到一个已记录区域闭合边界的最短距离。"""
        boundary = [
            xy for xy in (
                self._stored_point_xy(point)
                for point in ((group or {}).get("points") or [])
            )
            if xy is not None
        ]
        if not boundary:
            return None
        if len(boundary) == 1:
            return math.hypot(
                point_xy[0] - boundary[0][0],
                point_xy[1] - boundary[0][1],
            )
        distances = [
            self._point_to_segment_distance(
                point_xy,
                boundary[index],
                boundary[(index + 1) % len(boundary)],
            )
            for index in range(len(boundary))
        ]
        return min(distances)

    def _nearest_link_source_group(self, draft, sampled_point, fallback_group):
        """根据第一个连接点的位置确定连接桥真正连接的已有区域。

        用户可能在记录完区域2后驶回区域1，再从区域1记录通往区域3的
        连接桥。因此这里不能把当前编辑区域直接当作连接桥起点，而是比较
        连接点到每个已记录区域边界的距离，选择最近的区域。
        """
        point_xy = self._sample_point_xy(draft, sampled_point)
        if point_xy is None:
            return fallback_group, None
        candidates = []
        for group in draft.get("groups") or []:
            distance = self._group_boundary_distance(group, point_xy)
            if distance is None:
                continue
            candidates.append((
                distance,
                int(group.get("areaNumber") or 0),
                str(group.get("id") or ""),
                group,
            ))
        if not candidates:
            return fallback_group, None
        selected = min(candidates, key=lambda item: item[:3])
        return selected[3], selected[0]

    def _append_capture_event(self, model_id, draft, point_type, point):
        events = list(draft.get("captureSequence") or [])
        events.append({
            "sequence": len(events) + 1,
            "pointType": point_type,
            "pointId": point.get("id"),
        })
        draft["captureSequence"] = events
        return self.store.save_draft(model_id, draft)

    def _remove_capture_points(self, model_id, draft, point_ids):
        point_ids = set(point_ids or [])
        events = [
            event for event in (draft.get("captureSequence") or [])
            if event.get("pointId") not in point_ids
        ]
        for sequence, event in enumerate(events, start=1):
            event["sequence"] = sequence
        draft["captureSequence"] = events
        return self.store.save_draft(model_id, draft)

    def _reset_to_first_area_when_empty(self, model_id, draft, state):
        """Collapse an emptied multi-area draft back to its initial area.

        The two public "clear all" commands intentionally run independently, so
        the first command may leave the other point type in place.  Only after
        *both* area points and connection points are empty is the old multi-area
        topology no longer meaningful.  At that point keep the same modeling
        session/modelId, retain the original first group, and remove all derived
        groups, links, capture order, and unfinished planning output.

        Returning to area one here prevents the next RTK sample from being
        attached to the previously active area (for example area two).  Saved
        named tasks live outside this draft and are deliberately not touched.
        """
        groups = list(draft.get("groups") or [])
        links = list(draft.get("groupLinks") or [])
        area_point_count = sum(len(group.get("points") or []) for group in groups)
        link_point_count = sum(len(link.get("points") or []) for link in links)
        if area_point_count or link_point_count:
            return draft, state, False

        if groups:
            first_group = min(
                groups,
                key=lambda group: int(group.get("areaNumber") or 1),
            )
            first_group = dict(first_group)
            first_group["areaNumber"] = 1
            first_group["points"] = []
            first_group["subAreas"] = []
            first_group["connectors"] = []
            first_group["updatedAt"] = self._timestamp()
        else:
            created = self.store.create_group(model_id, None, now=self._timestamp())
            draft = created["draft"]
            first_group = dict(created["group"])

        draft["groups"] = [first_group]
        draft["groupLinks"] = []
        draft["captureSequence"] = []
        draft["recognition"] = {"confirmed": False, "items": []}
        draft["taskPreview"] = None
        draft["taskPlan"] = None
        saved = self.store.save_draft(model_id, draft, now=self._timestamp())

        state = dict(state)
        state["status"] = "recording"
        state["currentGroupId"] = first_group.get("id")
        state["currentLinkId"] = None
        state["pendingLinkNumber"] = None
        state["previousGroupId"] = None
        state["pendingGroupId"] = None
        state["captureMode"] = None
        state["currentPointType"] = "area"
        return saved, state, True

    def _summary(self, state, draft=None):
        if not state:
            return {
                "status": "idle",
                "areaPointCount": 0,
                "linkPointCount": 0,
                "totalAreaPointCount": 0,
                "totalLinkPointCount": 0,
                "groupCount": 0,
                "linkCount": 0,
            }
        draft = draft or self.store.get_draft(state.get("modelId"))
        groups = list(draft.get("groups") or [])
        links = list(draft.get("groupLinks") or [])
        current_group = self._find_group(draft, state.get("currentGroupId")) or {}
        current_link = self._find_link(draft, state.get("currentLinkId")) or {}
        return {
            "status": state.get("status") or "recording",
            "modelId": state.get("modelId"),
            "name": state.get("name") or draft.get("name") or "",
            "currentGroupId": state.get("currentGroupId"),
            "currentLinkId": state.get("currentLinkId"),
            "currentLinkNumber": (
                current_link.get("linkNumber")
                if current_link
                else state.get("pendingLinkNumber")
            ),
            "currentAreaNumber": current_group.get("areaNumber"),
            "currentPointType": state.get("currentPointType"),
            "areaPointCount": len(current_group.get("points") or []),
            "linkPointCount": len(current_link.get("points") or []),
            "totalAreaPointCount": sum(len(group.get("points") or []) for group in groups),
            "totalLinkPointCount": sum(len(link.get("points") or []) for link in links),
            "groupCount": len(groups),
            "linkCount": len(links),
            "createdAt": state.get("createdAt"),
            "updatedAt": state.get("updatedAt") or draft.get("updatedAt"),
        }

    def start(self, name=None, restart=False):
        with self._lock:
            current = self._read_state()
            if current and current.get("status") == "recording" and not restart:
                return self._summary(current)

            timestamp = self._timestamp()
            model = self.store.create_model(name or "modeling-{}".format(timestamp), now=timestamp)
            group = self.store.create_group(model["id"], None, now=timestamp)["group"]
            state = self._write_state({
                "status": "recording",
                "modelId": model["id"],
                "name": model.get("name"),
                "currentGroupId": group["id"],
                "currentLinkId": None,
                "pendingLinkNumber": None,
                "pendingGroupId": None,
                "captureMode": None,
                "currentPointType": "area",
                "createdAt": timestamp,
            })
            return self._summary(state)

    def current(self):
        with self._lock:
            state = self._read_state()
            return self._summary(state) if state else self._summary(None)

    def new_area(self):
        """Create and select the next area after its connection was recorded."""
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            current_group = self._find_group(draft, state.get("currentGroupId"))
            if current_group is None:
                raise ModelingSessionError("MODELING_GROUP_MISSING", "current modeling area is missing")
            if len(current_group.get("points") or []) < 4:
                raise ModelingSessionError(
                    "MODELING_AREA_INCOMPLETE",
                    "record at least four points before creating the next area",
                )

            current_link = self._find_link(draft, state.get("currentLinkId"))
            if current_link is None:
                raise ModelingSessionError(
                    "MODELING_LINK_MISSING",
                    "record connection points before creating the next area",
                )
            if current_link.get("endGroupId"):
                raise ModelingSessionError("MODELING_LINK_ALREADY_BOUND", "connection already has a destination area")
            if len(current_link.get("points") or []) < 2:
                raise ModelingSessionError(
                    "MODELING_LINK_INCOMPLETE",
                    "record at least two connection points before creating the next area",
                )
            source_group = self._find_group(draft, current_link.get("startGroupId"))
            if source_group is None:
                raise ModelingSessionError(
                    "MODELING_LINK_SOURCE_MISSING",
                    "connection source area is missing",
                )

            created = self.store.create_group_from_pending_link(
                model_id,
                current_link.get("id"),
                now=self._timestamp(),
            )
            next_group = created["group"]
            state["previousGroupId"] = source_group.get("id")
            state["currentGroupId"] = next_group.get("id")
            state["currentLinkId"] = None
            state["pendingLinkNumber"] = None
            state["pendingGroupId"] = None
            state["captureMode"] = None
            state["currentPointType"] = "area"
            state = self._write_state(state)
            summary = self._summary(state, created["draft"])
            return {
                "areaNumber": next_group.get("areaNumber"),
                "sourceAreaNumber": source_group.get("areaNumber"),
                "groupCount": summary.get("groupCount"),
            }

    def new_link(self):
        """Prepare a new connection bridge without consuming an RTK sample."""
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            current_group = self._find_group(draft, state.get("currentGroupId"))
            if current_group is None:
                raise ModelingSessionError("MODELING_GROUP_MISSING", "current modeling area is missing")
            if len(current_group.get("points") or []) < 4:
                raise ModelingSessionError(
                    "MODELING_AREA_INCOMPLETE",
                    "record at least four area points before creating a connection bridge",
                )

            active_link = self._find_link(draft, state.get("currentLinkId"))
            if active_link is not None:
                active_points = list(active_link.get("points") or [])
                if active_points:
                    raise ModelingSessionError(
                        "MODELING_NEW_AREA_REQUIRED",
                        "create the next area before creating another connection bridge",
                    )
                link_number = active_link.get("linkNumber") or len(draft.get("groupLinks") or [])
                state["captureMode"] = "link"
                state["currentPointType"] = "link"
                state["pendingLinkNumber"] = link_number
                state = self._write_state(state)
                return {
                    "modelId": model_id,
                    "linkNumber": link_number,
                    "linkPointCount": 0,
                    "session": self._summary(state, draft),
                }

            # Do not choose the source area yet. The first recorded bridge point selects
            # the nearest area, preserving branched layouts such as area 1 -> area 3.
            if state.get("captureMode") == "link" and state.get("pendingLinkNumber"):
                link_number = state.get("pendingLinkNumber")
            else:
                link_number = len(draft.get("groupLinks") or []) + 1
            state["currentLinkId"] = None
            state["pendingLinkNumber"] = link_number
            state["captureMode"] = "link"
            state["currentPointType"] = "link"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "linkNumber": link_number,
                "linkPointCount": 0,
                "session": self._summary(state, draft),
            }

    def record_area_point(self):
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            if (
                state.get("captureMode") == "link"
                and state.get("pendingLinkNumber")
                and not state.get("currentLinkId")
            ):
                raise ModelingSessionError(
                    "MODELING_LINK_POINT_REQUIRED",
                    "record connection points after creating a connection bridge",
                )
            active_link = self._find_link(draft, state.get("currentLinkId"))
            if active_link is not None and (active_link.get("points") or []):
                raise ModelingSessionError(
                    "MODELING_NEW_AREA_REQUIRED",
                    "create the next area before recording more area points",
                )

            group_id = state.get("currentGroupId")
            if not group_id:
                raise ModelingSessionError("MODELING_GROUP_MISSING", "current modeling area is missing")
            result = self.store.append_group_point(model_id, group_id, self._sample())
            saved = self._append_capture_event(
                model_id,
                result["draft"],
                "area",
                result["point"],
            )
            public_point = dict(result["point"])
            public_point["areaNumber"] = result["group"].get("areaNumber")
            state["currentPointType"] = "area"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "pointType": "area",
                "pointNo": result["point"].get("sequence"),
                "point": public_point,
                "session": self._summary(state, saved),
            }

    def record_link_point(self):
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            current_group = self._find_group(draft, state.get("currentGroupId"))
            if current_group is None:
                raise ModelingSessionError("MODELING_GROUP_MISSING", "current modeling area is missing")

            link_id = state.get("currentLinkId")
            source_group = current_group
            if not link_id:
                if len(current_group.get("points") or []) < 4:
                    raise ModelingSessionError(
                        "MODELING_AREA_INCOMPLETE",
                        "record at least four area points before recording connection points",
                    )
                # 先完成区域点数量校验再读取 RTK，避免失败命令无意义地
                # 消耗一次定位采样。这个点既用于保存，也用于判断桥从哪一区域出发。
                sampled_point = self._sample()
                source_group, _source_distance_cm = self._nearest_link_source_group(
                    draft,
                    sampled_point,
                    current_group,
                )
                created = self.store.create_pending_group_link(
                    model_id,
                    source_group["id"],
                    now=self._timestamp(),
                )
                link_id = created["groupLink"]["id"]
                state["currentLinkId"] = link_id
                state["pendingLinkNumber"] = None
                draft = created["draft"]
            else:
                sampled_point = self._sample()
                active_link = self._find_link(draft, link_id) or {}
                source_group = self._find_group(draft, active_link.get("startGroupId")) or current_group

            result = self.store.append_group_link_point(model_id, link_id, sampled_point)
            saved = self._append_capture_event(
                model_id,
                result["draft"],
                "link",
                result["point"],
            )
            state["currentPointType"] = "link"
            state["captureMode"] = "link"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "linkId": link_id,
                "linkNumber": (result.get("groupLink") or {}).get("linkNumber"),
                "pointType": "link",
                "pointNo": result["point"].get("sequence"),
                "point": result["point"],
                "sourceAreaNumber": source_group.get("areaNumber"),
                "session": self._summary(state, saved),
            }

    def undo(self, point_type=None):
        with self._lock:
            state = self._require_state()
            point_type = str(point_type or state.get("currentPointType") or "area").strip().lower()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            if point_type == "area":
                group = self._find_group(draft, state.get("currentGroupId"))
                points = list((group or {}).get("points") or [])
                if not points:
                    raise ModelingSessionError("MODELING_POINT_MISSING", "there is no area point to undo")
                result = self.store.delete_group_point(model_id, group["id"], points[-1]["id"])
                draft = self._remove_capture_points(model_id, result["draft"], [points[-1]["id"]])
            elif point_type == "link":
                link = self._find_link(draft, state.get("currentLinkId"))
                points = list((link or {}).get("points") or [])
                if not points:
                    raise ModelingSessionError("MODELING_POINT_MISSING", "there is no connection point to undo")
                result = self.store.delete_group_link_point(model_id, link["id"], points[-1]["id"])
                draft = self._remove_capture_points(model_id, result["draft"], [points[-1]["id"]])
            else:
                raise ModelingSessionError("MODELING_POINT_TYPE_INVALID", "pointType must be area or link")

            state["currentPointType"] = point_type
            state = self._write_state(state)
            return {
                "pointType": point_type,
                "session": self._summary(state, draft),
            }

    def delete_area_point(self, point_id):
        with self._lock:
            state = self._require_state()
            point_id = str(point_id or "").strip()
            if not point_id:
                raise ModelingSessionError("MODELING_POINT_ID_REQUIRED", "point id is required")

            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            group = next((
                item for item in (draft.get("groups") or [])
                if any(
                    point.get("id") == point_id
                    for point in (item.get("points") or [])
                )
            ), None)
            if group is None:
                raise ModelingSessionError("MODELING_POINT_NOT_FOUND", "area point not found")

            result = self.store.delete_group_point(model_id, group.get("id"), point_id)
            saved = self._remove_capture_points(model_id, result["draft"], [point_id])
            state["currentPointType"] = "area"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "groupId": group.get("id"),
                "pointType": "area",
                "id": point_id,
                "session": self._summary(state, saved),
            }

    def delete_link_point(self, point_id):
        with self._lock:
            state = self._require_state()
            point_id = str(point_id or "").strip()
            if not point_id:
                raise ModelingSessionError("MODELING_POINT_ID_REQUIRED", "point id is required")

            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            link = next((
                item for item in (draft.get("groupLinks") or [])
                if any(
                    point.get("id") == point_id
                    for point in (item.get("points") or [])
                )
            ), None)
            if link is None:
                raise ModelingSessionError("MODELING_POINT_NOT_FOUND", "connection point not found")

            result = self.store.delete_group_link_point(model_id, link.get("id"), point_id)
            saved = self._remove_capture_points(model_id, result["draft"], [point_id])
            state["currentPointType"] = "link"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "linkId": link.get("id"),
                "pointType": "link",
                "id": point_id,
                "session": self._summary(state, saved),
            }

    def clear(self, point_type=None):
        with self._lock:
            state = self._require_state()
            point_type = str(point_type or state.get("currentPointType") or "area").strip().lower()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            if point_type == "area":
                group = self._find_group(draft, state.get("currentGroupId")) or {}
                removed_ids = [point.get("id") for point in (group.get("points") or [])]
                result = self.store.clear_group_points(model_id, state.get("currentGroupId"))
            elif point_type == "link":
                if not state.get("currentLinkId"):
                    raise ModelingSessionError("MODELING_LINK_MISSING", "current connection is missing")
                link = self._find_link(draft, state.get("currentLinkId")) or {}
                removed_ids = [point.get("id") for point in (link.get("points") or [])]
                result = self.store.clear_group_link_points(model_id, state.get("currentLinkId"))
            else:
                raise ModelingSessionError("MODELING_POINT_TYPE_INVALID", "pointType must be area or link")

            saved = self._remove_capture_points(model_id, result["draft"], removed_ids)
            state["currentPointType"] = point_type
            state = self._write_state(state)
            return {
                "pointType": point_type,
                "session": self._summary(state, saved),
            }

    def clear_all(self, point_type):
        with self._lock:
            state = self._require_state()
            point_type = str(point_type or "").strip().lower()
            if point_type not in ("area", "link"):
                raise ModelingSessionError(
                    "MODELING_POINT_TYPE_INVALID",
                    "pointType must be area or link",
                )

            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            removed_ids = []
            if point_type == "area":
                items = list(draft.get("groups") or [])
                for group in items:
                    removed_ids.extend(
                        point.get("id")
                        for point in (group.get("points") or [])
                        if point.get("id")
                    )
                    result = self.store.clear_group_points(model_id, group.get("id"))
                    draft = result["draft"]
            else:
                items = list(draft.get("groupLinks") or [])
                for link in items:
                    removed_ids.extend(
                        point.get("id")
                        for point in (link.get("points") or [])
                        if point.get("id")
                    )
                    result = self.store.clear_group_link_points(model_id, link.get("id"))
                    draft = result["draft"]

            saved = self._remove_capture_points(model_id, draft, removed_ids)
            saved, state, reset_to_first_area = self._reset_to_first_area_when_empty(
                model_id,
                saved,
                state,
            )
            if not reset_to_first_area:
                state["currentPointType"] = point_type
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "pointType": point_type,
                "clearedPointCount": len(removed_ids),
                "resetToFirstArea": reset_to_first_area,
                "session": self._summary(state, saved),
            }

    def _prepare_route_planning(self, state):
        """校验并识别当前建模数据，使其具备生成路线的条件。

        ``finish_modeling`` 和 ``replan_modeling_route`` 都可能成为一次建模
        的首个规划命令。无论由哪个命令触发，生成预览前都必须完成相同
        的准备步骤：解析兼容模式记录、校验区域与连接桥、识别每个区域。

        调用方必须已经持有 ``self._lock``，并传入活动会话状态。
        """
        model_id = state["modelId"]
        draft = self.store.get_draft(model_id)

        # 兼容早期把区域点和连接点混合记录在同一条边界序列中的模型。
        if state.get("captureMode") == "mixed_boundary":
            try:
                draft = resolve_mixed_capture(draft)
            except MixedCaptureError as error:
                raise ModelingSessionError(error.code, error.message)
            draft = self.store.save_draft(model_id, draft)

        # 每条连接桥至少保留起点和终点，也可以包含任意多个桥内途经点，
        # 并且必须明确连接哪两个区域。
        links = list(draft.get("groupLinks") or [])
        incomplete_links = [
            link for link in links
            if len(link.get("points") or []) < 2
            or not link.get("startGroupId")
            or not link.get("endGroupId")
        ]
        if incomplete_links:
            raise ModelingSessionError(
                "MODELING_LINK_INCOMPLETE",
                "each connection must contain at least its original start and end points",
            )

        # 每个区域至少四个记录点，才能形成可识别的闭合清扫区域。
        groups = list(draft.get("groups") or [])
        incomplete_groups = [group for group in groups if len(group.get("points") or []) < 4]
        if incomplete_groups:
            raise ModelingSessionError(
                "MODELING_AREA_INCOMPLETE",
                "each modeling area requires at least four points",
            )

        # 识别结果会写回各区域的 subAreas，路径预览以这些结果生成清扫线。
        for group in groups:
            recognized = self.store.recognize_group(model_id, group["id"])
            recognition = recognized.get("recognition") or {}
            if recognition.get("needsConfirmation") or not recognized.get("group", {}).get("subAreas"):
                raise ModelingSessionError(
                    "MODELING_RECOGNITION_NEEDS_CONFIRMATION",
                    "the recorded area needs confirmation before a path can be generated",
                )

        return self.store.get_draft(model_id)

    def finish(self):
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            self._prepare_route_planning(state)

            self.store.build_task_preview(model_id)
            generated = self.store.generate_task_plan(model_id)
            saved = self.store.save_model(model_id)
            state["status"] = "ready"
            state["currentPointType"] = None
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "areaOrder": generated["taskPlan"].get("areaOrder") or [],
                "taskName": generated["taskPlan"].get("taskName") or saved.get("name") or "",
                "updatedAt": saved.get("updatedAt"),
                "taskPreview": saved.get("taskPreview"),
                "taskPlan": generated["taskPlan"],
                "session": self._summary(state, saved),
            }

    def replan(self, area_order):
        """使用已记录的区域和连接桥，按新的区域顺序重新生成当前路线。"""
        with self._lock:
            state = self._require_state(allow_ready=True)
            if not isinstance(area_order, (list, tuple)):
                raise ModelingSessionError("MODELING_AREA_ORDER_INVALID", "areaOrder must be an array")
            model_id = state["modelId"]
            # 新前端把本命令作为首次规划入口，因此这里不能假定
            # finish_modeling 已经提前完成区域识别和完整性校验。
            draft = self._prepare_route_planning(state)
            route_policy = dict(draft.get("routePolicy") or {})
            route_policy["type"] = "area_order"
            route_policy["areaOrder"] = list(area_order)
            route_policy.pop("forceEvenLanes", None)
            draft["routePolicy"] = route_policy
            draft["taskPreview"] = None
            draft["taskPlan"] = None
            self.store.save_draft(model_id, draft, now=self._timestamp())
            self.store.build_task_preview(model_id)
            generated = self.store.generate_task_plan(model_id)
            saved = self.store.save_model(model_id)
            state["status"] = "ready"
            state["currentPointType"] = None
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "areaOrder": generated["taskPlan"].get("areaOrder") or [],
                "taskName": generated["taskPlan"].get("taskName") or saved.get("name") or "",
                "updatedAt": saved.get("updatedAt"),
                "taskPreview": saved.get("taskPreview"),
                "taskPlan": generated["taskPlan"],
                "session": self._summary(state, saved),
            }

    def current_path(self):
        with self._lock:
            state = self._require_state(allow_ready=True)
            draft = self.store.get_draft(state["modelId"])
            task_plan = draft.get("taskPlan")
            if not isinstance(task_plan, dict) or task_plan.get("status") != "ready":
                raise ModelingSessionError("MODELING_PATH_NOT_READY", "modeling task plan is not ready")
            return {
                "modelId": draft.get("id") or state["modelId"],
                "areaOrder": task_plan.get("areaOrder") or [],
                "taskName": task_plan.get("taskName") or draft.get("name") or "",
                "updatedAt": draft.get("updatedAt") or task_plan.get("generatedAt"),
                "taskPreview": draft.get("taskPreview"),
                "taskPlan": task_plan,
            }
