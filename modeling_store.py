# coding=utf-8
import io
import json
import os
import re
import time
import uuid

from modeling_preview import build_model_preview
from modeling_recognition import recognize_group_points
from modeling_task_generator import generate_task_plan
from modeling_coordinates import normalize_draft_coordinates


MODEL_SCHEMA_VERSION = 1
MODEL_ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")

try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


def _text(value):
    if isinstance(value, text_type):
        return value
    if isinstance(value, binary_type):
        return value.decode("utf-8")
    return text_type(value)


def _apply_route_selections_to_preview(preview, task_plan):
    """让前端预览中的 lanes 与机器人最终选择的奇偶方案保持一致。"""
    if not isinstance(preview, dict) or not isinstance(task_plan, dict):
        return preview
    selections = {
        item.get("groupId"): item
        for item in (task_plan.get("routeSelections") or [])
        if isinstance(item, dict) and item.get("groupId")
    }
    total_lane_count = 0
    for group in preview.get("groups") or []:
        selection = selections.get(group.get("groupId")) or {}
        sub_area_selections = {
            item.get("subAreaId"): item
            for item in (selection.get("subAreas") or [])
            if isinstance(item, dict) and item.get("subAreaId")
        }
        for sub_area in group.get("subAreas") or []:
            selected = sub_area_selections.get(sub_area.get("id"))
            if selected:
                selected_count = int(selected.get("laneCount") or 0)
                candidate = next(
                    (
                        item for item in (sub_area.get("laneCandidates") or [])
                        if int(item.get("laneCount") or 0) == selected_count
                    ),
                    None,
                )
                if candidate:
                    sub_area["lanes"] = list(candidate.get("lanes") or [])
                    sub_area["laneCount"] = len(sub_area["lanes"])
                    sub_area["laneSpacingCm"] = candidate.get("laneSpacingCm")
                    sub_area["actualOverlapCm"] = candidate.get("actualOverlapCm")
            total_lane_count += int(sub_area.get("laneCount") or 0)
    preview["areaOrder"] = list(task_plan.get("areaOrder") or preview.get("areaOrder") or [])
    summary = dict(preview.get("summary") or {})
    summary["laneCount"] = total_lane_count
    preview["summary"] = summary
    return preview


class ModelingStoreError(Exception):
    pass


class ModelNotFoundError(ModelingStoreError):
    pass


class InvalidModelIdError(ModelingStoreError):
    pass


class InvalidModelPayloadError(ModelingStoreError):
    pass


class ModelingStore(object):
    def __init__(self, root_dir, now=None):
        self.root_dir = os.path.abspath(root_dir)
        self.models_dir = os.path.join(self.root_dir, "models")
        self.drafts_dir = os.path.join(self.root_dir, "drafts")
        self._now = now or time.time
        self._ensure_dirs()

    def _ensure_dirs(self):
        for path in (self.root_dir, self.models_dir, self.drafts_dir):
            if not os.path.exists(path):
                os.makedirs(path)

    def _timestamp(self, override=None):
        return int(override if override is not None else self._now())

    def _validate_model_id(self, model_id):
        model_id = str(model_id or "")
        if not model_id or not MODEL_ID_PATTERN.match(model_id):
            raise InvalidModelIdError("invalid model id")
        return model_id

    def _model_path(self, model_id):
        return os.path.join(self.models_dir, self._validate_model_id(model_id) + ".json")

    def _draft_path(self, model_id):
        return os.path.join(self.drafts_dir, self._validate_model_id(model_id) + ".json")

    def _read_json(self, path):
        if not os.path.exists(path):
            raise ModelNotFoundError("model not found")
        with io.open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)

    def _write_json(self, path, payload):
        tmp_path = path + ".tmp"
        serialized = json.dumps(payload, ensure_ascii=True, indent=2, sort_keys=True)
        if not isinstance(serialized, type(u"")):
            serialized = serialized.decode("utf-8")
        with io.open(tmp_path, "w", encoding="utf-8") as handle:
            handle.write(serialized)
        getattr(os, "replace", os.rename)(tmp_path, path)

    def _new_model_id(self):
        return uuid.uuid4().hex[:12]

    def _new_group_id(self):
        return "g" + uuid.uuid4().hex[:11]

    def _new_group_link_id(self):
        return "l" + uuid.uuid4().hex[:11]

    def _new_point_id(self):
        return "p" + uuid.uuid4().hex[:11]

    def _normalize_optional_float(self, value):
        if value is None or value == "":
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            raise InvalidModelPayloadError("invalid numeric value")

    def _normalize_required_float(self, value, field_name):
        normalized = self._normalize_optional_float(value)
        if normalized is None:
            raise InvalidModelPayloadError("{} is required".format(field_name))
        return normalized

    def _next_area_number(self, groups):
        numbers = []
        for group in groups:
            try:
                numbers.append(int(group.get("areaNumber") or 0))
            except (TypeError, ValueError):
                pass
        return (max(numbers) if numbers else 0) + 1

    def _next_link_number(self, links):
        """Return the next stable, user-facing connection number."""
        numbers = []
        for index, link in enumerate(links or [], start=1):
            try:
                numbers.append(int(link.get("linkNumber") or index))
            except (TypeError, ValueError):
                numbers.append(index)
        return (max(numbers) if numbers else 0) + 1

    def _normalize_group_link_point_roles(self, points, now):
        """Renumber a connection polyline and mark its start, middle and end points."""
        point_count = len(points or [])
        normalized_points = []
        for sequence, raw_point in enumerate(points or [], start=1):
            point = dict(raw_point)
            point["sequence"] = sequence
            if sequence == 1:
                point["role"] = "group_link_start"
            elif sequence == point_count:
                point["role"] = "group_link_end"
            else:
                point["role"] = "group_link_waypoint"
            point["roles"] = ["group_connector"]
            point["updatedAt"] = now
            normalized_points.append(point)
        return normalized_points

    def _normalize_group(self, payload, group_id=None, area_number=None, now=None):
        if not isinstance(payload, dict):
            raise InvalidModelPayloadError("group payload must be an object")
        current_time = self._timestamp(now)
        normalized = dict(payload)
        normalized["id"] = self._validate_model_id(group_id or normalized.get("id") or self._new_group_id())
        normalized["areaNumber"] = int(normalized.get("areaNumber") or area_number or 1)
        normalized["name"] = _text(normalized.get("name") or "区域组{}".format(normalized["areaNumber"]))
        direction = str(normalized.get("sweepDirection") or "auto")
        if direction not in ("auto", "manual"):
            raise InvalidModelPayloadError("invalid sweep direction")
        normalized["sweepDirection"] = direction
        normalized["sweepAngle"] = self._normalize_optional_float(normalized.get("sweepAngle"))
        if direction == "auto":
            normalized["sweepAngle"] = None
        normalized["createdAt"] = int(normalized.get("createdAt") or current_time)
        normalized["updatedAt"] = current_time
        normalized.setdefault("points", [])
        normalized.setdefault("subAreas", [])
        normalized.setdefault("connectors", [])
        if not isinstance(normalized["points"], list):
            raise InvalidModelPayloadError("group points must be a list")
        return normalized

    def _find_group_index(self, draft, group_id):
        group_id = self._validate_model_id(group_id)
        groups = draft.get("groups") or []
        for index, group in enumerate(groups):
            if group.get("id") == group_id:
                return index
        raise ModelNotFoundError("group not found")

    def _find_group_link_index(self, draft, link_id):
        link_id = self._validate_model_id(link_id)
        links = draft.get("groupLinks") or []
        for index, link in enumerate(links):
            if link.get("id") == link_id:
                return index
        raise ModelNotFoundError("group link not found")

    def _normalize_model(self, payload, model_id=None, now=None, status=None):
        if not isinstance(payload, dict):
            raise InvalidModelPayloadError("model payload must be an object")

        current_time = self._timestamp(now)
        normalized = dict(payload)
        normalized["id"] = self._validate_model_id(model_id or normalized.get("id") or self._new_model_id())
        normalized["name"] = _text(normalized.get("name") or "untitled-model")
        normalized["version"] = int(normalized.get("version") or MODEL_SCHEMA_VERSION)
        normalized["status"] = status or str(normalized.get("status") or "draft")
        normalized["createdAt"] = int(normalized.get("createdAt") or current_time)
        normalized["updatedAt"] = current_time
        normalized.setdefault("groups", [])
        normalized.setdefault("groupLinks", [])
        normalized.setdefault("recognition", {"confirmed": False, "items": []})
        normalized.setdefault("taskPreview", None)
        normalized.setdefault("taskPlan", None)
        return normalized

    def _clear_task_outputs(self, draft):
        draft["taskPreview"] = None
        draft["taskPlan"] = None

    def create_model(self, name, now=None):
        current_time = self._timestamp(now)
        model = self._normalize_model({
            "name": name,
            "createdAt": current_time,
            "updatedAt": current_time,
            "status": "draft",
        }, now=current_time, status="draft")
        self._write_json(self._model_path(model["id"]), model)
        self._write_json(self._draft_path(model["id"]), model)
        return model

    def list_models(self):
        self._ensure_dirs()
        items = []
        for filename in sorted(os.listdir(self.models_dir)):
            if not filename.endswith(".json"):
                continue
            model = self._read_json(os.path.join(self.models_dir, filename))
            items.append({
                "id": model.get("id"),
                "name": model.get("name"),
                "status": model.get("status"),
                "version": model.get("version"),
                "createdAt": model.get("createdAt"),
                "updatedAt": model.get("updatedAt"),
                "savedAt": model.get("savedAt"),
            })
        return items

    def get_model(self, model_id):
        return self._read_json(self._model_path(model_id))

    def get_draft(self, model_id):
        return self._read_json(self._draft_path(model_id))

    def save_draft(self, model_id, draft, now=None):
        existing = self.get_model(model_id)
        normalized = self._normalize_model(draft, model_id=existing["id"], now=now, status="draft")
        normalized["createdAt"] = existing.get("createdAt") or normalized["createdAt"]
        self._write_json(self._draft_path(model_id), normalized)
        return normalized

    def save_model(self, model_id, now=None):
        draft = self.get_draft(model_id)
        saved_at = self._timestamp(now)
        model = self._normalize_model(draft, model_id=model_id, now=saved_at, status="saved")
        model["savedAt"] = saved_at
        self._write_json(self._model_path(model_id), model)
        self._write_json(self._draft_path(model_id), model)
        return model

    def build_task_preview(self, model_id, now=None):
        current_time = self._timestamp(now)
        # Re-normalize before geometry generation so an older draft whose
        # second area started again at (0, 0) is repaired automatically.
        draft = self.get_draft(model_id)
        force_coordinate_migration = (
            (draft.get("coordinateFrame") or {}).get("type") != "model_origin"
        )
        draft = normalize_draft_coordinates(draft, force=force_coordinate_migration)
        preview = build_model_preview(draft, now=current_time)
        draft["taskPreview"] = preview
        draft["taskPlan"] = None
        saved = self.save_draft(model_id, draft, now=current_time)
        return {
            "draft": saved,
            "taskPreview": saved.get("taskPreview"),
        }

    def generate_task_plan(self, model_id, now=None):
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        task_plan = generate_task_plan(draft, now=current_time)
        draft["taskPreview"] = _apply_route_selections_to_preview(
            draft.get("taskPreview"),
            task_plan,
        )
        draft["taskPlan"] = task_plan
        saved = self.save_draft(model_id, draft, now=current_time)
        return {
            "draft": saved,
            "taskPlan": saved.get("taskPlan"),
        }

    def create_group(self, model_id, name=None, now=None):
        draft = self.get_draft(model_id)
        groups = list(draft.get("groups") or [])
        group = self._normalize_group({
            "name": name,
        }, area_number=self._next_area_number(groups), now=now)
        groups.append(group)
        draft["groups"] = groups
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=now)
        index = self._find_group_index(saved, group["id"])
        return {
            "draft": saved,
            "group": saved["groups"][index],
        }

    def update_group(self, model_id, group_id, payload, now=None):
        if not isinstance(payload, dict):
            raise InvalidModelPayloadError("group payload must be an object")
        draft = self.get_draft(model_id)
        index = self._find_group_index(draft, group_id)
        group = dict(draft["groups"][index])
        for key in ("name", "sweepDirection", "sweepAngle"):
            if key in payload:
                group[key] = payload[key]
        group = self._normalize_group(group, group_id=group_id, area_number=group.get("areaNumber"), now=now)
        draft["groups"][index] = group
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=now)
        saved_index = self._find_group_index(saved, group_id)
        return {
            "draft": saved,
            "group": saved["groups"][saved_index],
        }

    def clear_group_points(self, model_id, group_id, now=None):
        draft = self.get_draft(model_id)
        index = self._find_group_index(draft, group_id)
        group = dict(draft["groups"][index])
        group["points"] = []
        group["subAreas"] = []
        group["connectors"] = []
        group["updatedAt"] = self._timestamp(now)
        draft["groups"][index] = group
        recognition = draft.get("recognition")
        if isinstance(recognition, dict):
            recognition["confirmed"] = False
            draft["recognition"] = recognition
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=now)
        saved_index = self._find_group_index(saved, group_id)
        return {
            "draft": saved,
            "group": saved["groups"][saved_index],
        }

    def delete_group(self, model_id, group_id, now=None):
        draft = self.get_draft(model_id)
        index = self._find_group_index(draft, group_id)
        del draft["groups"][index]
        draft["groupLinks"] = [
            link for link in (draft.get("groupLinks") or [])
            if link.get("startGroupId") != group_id and link.get("endGroupId") != group_id
        ]
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=now)
        return {
            "draft": saved,
            "groupId": group_id,
        }

    def create_group_link(self, model_id, payload, now=None):
        if not isinstance(payload, dict):
            raise InvalidModelPayloadError("group link payload must be an object")
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        groups = draft.get("groups") or []
        start_index = self._find_group_index(draft, payload.get("startGroupId"))
        end_index = self._find_group_index(draft, payload.get("endGroupId"))
        start_group = groups[start_index]
        end_group = groups[end_index]
        start_group_id = start_group.get("id")
        end_group_id = end_group.get("id")
        if start_group_id == end_group_id:
            raise InvalidModelPayloadError("group link requires two different groups")

        links = list(draft.get("groupLinks") or [])
        link = {
            "id": self._validate_model_id(payload.get("id") or self._new_group_link_id()),
            "linkNumber": self._next_link_number(links),
            "type": "group_connector",
            "name": _text(payload.get("name") or u"{} -> {}".format(start_group.get("name"), end_group.get("name"))),
            "startGroupId": start_group_id,
            "endGroupId": end_group_id,
            "points": [],
            "status": "draft",
            "createdAt": current_time,
            "updatedAt": current_time,
        }
        links.append(link)
        draft["groupLinks"] = links
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_link_index(saved, link["id"])
        return {
            "draft": saved,
            "groupLink": saved["groupLinks"][saved_index],
        }

    def create_pending_group_link(self, model_id, start_group_id, now=None):
        """Create a connection whose destination will be the next explicit area."""
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        groups = draft.get("groups") or []
        start_index = self._find_group_index(draft, start_group_id)
        start_group = groups[start_index]
        links = list(draft.get("groupLinks") or [])
        link = {
            "id": self._new_group_link_id(),
            "linkNumber": self._next_link_number(links),
            "type": "group_connector",
            "name": _text(u"{} -> 待新增区域".format(start_group.get("name"))),
            "startGroupId": start_group.get("id"),
            "endGroupId": None,
            "points": [],
            "status": "draft",
            "createdAt": current_time,
            "updatedAt": current_time,
        }
        links.append(link)
        draft["groupLinks"] = links
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_link_index(saved, link["id"])
        return {
            "draft": saved,
            "groupLink": saved["groupLinks"][saved_index],
        }

    def create_group_from_pending_link(self, model_id, link_id, name=None, now=None):
        """Atomically create the next area and bind the pending connection."""
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        link_index = self._find_group_link_index(draft, link_id)
        link = dict(draft["groupLinks"][link_index])
        start_group_index = self._find_group_index(draft, link.get("startGroupId"))
        start_group = draft["groups"][start_group_index]

        if link.get("endGroupId"):
            raise InvalidModelPayloadError("group link is already bound")
        if len(link.get("points") or []) < 2:
            raise InvalidModelPayloadError("record at least two connection points before creating the next area")

        groups = list(draft.get("groups") or [])
        end_group = self._normalize_group(
            {"name": name},
            area_number=self._next_area_number(groups),
            now=current_time,
        )
        groups.append(end_group)
        draft["groups"] = groups

        link["endGroupId"] = end_group.get("id")
        link["name"] = _text(u"{} -> {}".format(start_group.get("name"), end_group.get("name")))
        link["status"] = "ready"
        link["updatedAt"] = current_time
        draft["groupLinks"][link_index] = link
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=current_time)
        saved_link_index = self._find_group_link_index(saved, link_id)
        saved_group_index = self._find_group_index(saved, end_group.get("id"))
        return {
            "draft": saved,
            "groupLink": saved["groupLinks"][saved_link_index],
            "group": saved["groups"][saved_group_index],
        }

    def append_group_link_point(self, model_id, link_id, point, now=None):
        if not isinstance(point, dict):
            raise InvalidModelPayloadError("point payload must be an object")
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        index = self._find_group_link_index(draft, link_id)
        link = dict(draft["groupLinks"][index])
        points = list(link.get("points") or [])

        normalized = dict(point)
        normalized["id"] = self._validate_model_id(normalized.get("id") or self._new_point_id())
        normalized["sequence"] = len(points) + 1
        normalized["lat"] = self._normalize_required_float(normalized.get("lat"), "lat")
        normalized["lon"] = self._normalize_required_float(normalized.get("lon"), "lon")
        normalized["heading"] = self._normalize_optional_float(normalized.get("heading"))
        normalized["x"] = self._normalize_optional_float(normalized.get("x"))
        normalized["y"] = self._normalize_optional_float(normalized.get("y"))
        normalized["source"] = str(normalized.get("source") or "rtk_mean")
        normalized["createdAt"] = int(normalized.get("createdAt") or current_time)
        normalized["updatedAt"] = current_time

        points.append(normalized)
        points = self._normalize_group_link_point_roles(points, current_time)
        link["points"] = points
        link["status"] = "ready" if len(points) >= 2 and link.get("endGroupId") else "draft"
        link["updatedAt"] = current_time
        draft["groupLinks"][index] = link
        # Connection points use the same origin immediately, so querying the
        # active model never exposes a second or missing coordinate frame.
        draft = normalize_draft_coordinates(draft)
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_link_index(saved, link_id)
        saved_link = saved["groupLinks"][saved_index]
        return {
            "draft": saved,
            "groupLink": saved_link,
            "point": saved_link["points"][-1],
        }

    def delete_group_link_point(self, model_id, link_id, point_id, now=None):
        point_id = self._validate_model_id(point_id)
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        index = self._find_group_link_index(draft, link_id)
        link = dict(draft["groupLinks"][index])
        points = list(link.get("points") or [])
        next_points = [point for point in points if point.get("id") != point_id]
        if len(next_points) == len(points):
            raise ModelNotFoundError("point not found")

        next_points = self._normalize_group_link_point_roles(next_points, current_time)
        link["points"] = next_points
        link["status"] = "ready" if len(next_points) >= 2 and link.get("endGroupId") else "draft"
        link["updatedAt"] = current_time
        draft["groupLinks"][index] = link
        draft = normalize_draft_coordinates(draft)
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_link_index(saved, link_id)
        return {
            "draft": saved,
            "groupLink": saved["groupLinks"][saved_index],
            "pointId": point_id,
        }

    def clear_group_link_points(self, model_id, link_id, now=None):
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        index = self._find_group_link_index(draft, link_id)
        link = dict(draft["groupLinks"][index])
        link["points"] = []
        link["status"] = "draft"
        link["updatedAt"] = current_time
        draft["groupLinks"][index] = link
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_link_index(saved, link_id)
        return {
            "draft": saved,
            "groupLink": saved["groupLinks"][saved_index],
        }

    def delete_group_link(self, model_id, link_id, now=None):
        draft = self.get_draft(model_id)
        index = self._find_group_link_index(draft, link_id)
        del draft["groupLinks"][index]
        self._clear_task_outputs(draft)
        saved = self.save_draft(model_id, draft, now=now)
        return {
            "draft": saved,
            "groupLinkId": link_id,
        }

    def append_group_point(self, model_id, group_id, point, now=None):
        if not isinstance(point, dict):
            raise InvalidModelPayloadError("point payload must be an object")
        draft = self.get_draft(model_id)
        index = self._find_group_index(draft, group_id)
        group = dict(draft["groups"][index])
        points = list(group.get("points") or [])
        if not isinstance(points, list):
            raise InvalidModelPayloadError("group points must be a list")

        current_time = self._timestamp(now)
        normalized = dict(point)
        normalized["id"] = self._validate_model_id(normalized.get("id") or self._new_point_id())
        normalized["sequence"] = len(points) + 1
        normalized["lat"] = self._normalize_required_float(normalized.get("lat"), "lat")
        normalized["lon"] = self._normalize_required_float(normalized.get("lon"), "lon")
        normalized["heading"] = self._normalize_optional_float(normalized.get("heading"))
        normalized["x"] = self._normalize_optional_float(normalized.get("x"))
        normalized["y"] = self._normalize_optional_float(normalized.get("y"))
        normalized["source"] = str(normalized.get("source") or "rtk_mean")
        normalized["areaNumber"] = int(group.get("areaNumber") or 1)
        normalized.setdefault("role", "unknown")
        normalized.setdefault("roles", [])
        normalized["createdAt"] = int(normalized.get("createdAt") or current_time)
        normalized["updatedAt"] = current_time

        points.append(normalized)
        group["points"] = points
        group["updatedAt"] = current_time
        draft["groups"][index] = group
        # Keep the point list returned to the mini app in the same global frame
        # that will later be used by preview and executable task generation.
        draft = normalize_draft_coordinates(draft)
        recognition = draft.get("recognition")
        if isinstance(recognition, dict):
            recognition["confirmed"] = False
            draft["recognition"] = recognition
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=now)
        saved_index = self._find_group_index(saved, group_id)
        saved_group = saved["groups"][saved_index]
        return {
            "draft": saved,
            "group": saved_group,
            "point": saved_group["points"][-1],
        }

    def delete_group_point(self, model_id, group_id, point_id, now=None):
        point_id = self._validate_model_id(point_id)
        draft = self.get_draft(model_id)
        index = self._find_group_index(draft, group_id)
        group = dict(draft["groups"][index])
        points = list(group.get("points") or [])
        if not isinstance(points, list):
            raise InvalidModelPayloadError("group points must be a list")

        next_points = [point for point in points if point.get("id") != point_id]
        if len(next_points) == len(points):
            raise ModelNotFoundError("point not found")
        for sequence, point in enumerate(next_points, start=1):
            point["sequence"] = sequence
            point["updatedAt"] = self._timestamp(now)

        group["points"] = next_points
        group["updatedAt"] = self._timestamp(now)
        group["subAreas"] = []
        group["connectors"] = []
        draft["groups"][index] = group
        recognition = draft.get("recognition")
        if isinstance(recognition, dict):
            recognition["confirmed"] = False
            draft["recognition"] = recognition
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=now)
        saved_index = self._find_group_index(saved, group_id)
        return {
            "draft": saved,
            "group": saved["groups"][saved_index],
            "pointId": point_id,
        }

    def recognize_group(self, model_id, group_id, now=None):
        current_time = self._timestamp(now)
        # This also migrates already-recorded legacy models before recognizing
        # area two/three.  Recognition must never create a per-area origin.
        draft = self.get_draft(model_id)
        force_coordinate_migration = (
            (draft.get("coordinateFrame") or {}).get("type") != "model_origin"
        )
        draft = normalize_draft_coordinates(draft, force=force_coordinate_migration)
        index = self._find_group_index(draft, group_id)
        group = dict(draft["groups"][index])
        recognition = recognize_group_points(group)

        group["points"] = recognition["points"]
        group["subAreas"] = recognition["subAreas"]
        group["connectors"] = recognition["connectors"]
        group["updatedAt"] = current_time
        draft["groups"][index] = group
        auto_confirmed = recognition.get("status") == "recognized" and recognition.get("needsConfirmation") is False
        draft["recognition"] = {
            "confirmed": bool(auto_confirmed),
            "generatedAt": current_time,
            "confirmedAt": current_time if auto_confirmed else None,
            "groupId": group_id,
            "status": recognition["status"],
            "message": recognition["message"],
            "needsConfirmation": recognition["needsConfirmation"],
            "summary": recognition["summary"],
            "warnings": recognition["warnings"],
        }
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_index(saved, group_id)
        return {
            "draft": saved,
            "group": saved["groups"][saved_index],
            "recognition": saved["recognition"],
        }

    def confirm_group_recognition(self, model_id, group_id, now=None):
        current_time = self._timestamp(now)
        draft = self.get_draft(model_id)
        index = self._find_group_index(draft, group_id)
        group = dict(draft["groups"][index])
        recognition = draft.get("recognition")
        if not isinstance(recognition, dict) or recognition.get("groupId") != group_id:
            raise InvalidModelPayloadError("recognition result is missing")
        if recognition.get("status") == "insufficient_points":
            raise InvalidModelPayloadError("insufficient points cannot be confirmed")
        if not group.get("subAreas"):
            raise InvalidModelPayloadError("recognized sub areas are missing")

        recognition = dict(recognition)
        recognition["confirmed"] = True
        recognition["confirmedAt"] = current_time
        recognition["groupId"] = group_id
        draft["recognition"] = recognition
        self._clear_task_outputs(draft)

        saved = self.save_draft(model_id, draft, now=current_time)
        saved_index = self._find_group_index(saved, group_id)
        return {
            "draft": saved,
            "group": saved["groups"][saved_index],
            "recognition": saved["recognition"],
        }

    def delete_model(self, model_id):
        model_path = self._model_path(model_id)
        draft_path = self._draft_path(model_id)
        removed = False
        for path in (model_path, draft_path):
            if os.path.exists(path):
                os.remove(path)
                removed = True
        if not removed:
            raise ModelNotFoundError("model not found")
        return True
