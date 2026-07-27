# coding=utf-8
import io
import json
import os
import threading
import time

from modeling_capture import MixedCaptureError, resolve_mixed_capture


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

    def record_area_point(self):
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            active_link = self._find_link(draft, state.get("currentLinkId"))
            if active_link is not None:
                if active_link.get("status") != "ready":
                    raise ModelingSessionError(
                        "MODELING_LINK_INCOMPLETE",
                        "record both connection points before recording the next area",
                    )
                if state.get("captureMode") != "mixed_boundary":
                    state["currentGroupId"] = state.get("pendingGroupId")
                    state["currentLinkId"] = None
                    state["pendingGroupId"] = None

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
            state["currentPointType"] = "area"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "groupId": group_id,
                "pointType": "area",
                "pointNo": result["point"].get("sequence"),
                "point": result["point"],
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
            if not link_id:
                if len(current_group.get("points") or []) < 4:
                    state["captureMode"] = "mixed_boundary"
                next_group = self.store.create_group(model_id, None)["group"]
                created = self.store.create_group_link(model_id, {
                    "startGroupId": current_group["id"],
                    "endGroupId": next_group["id"],
                })
                link_id = created["groupLink"]["id"]
                state["currentLinkId"] = link_id
                state["pendingGroupId"] = next_group["id"]
                draft = created["draft"]

            result = self.store.append_group_link_point(model_id, link_id, self._sample())
            saved = self._append_capture_event(
                model_id,
                result["draft"],
                "link",
                result["point"],
            )
            state["currentPointType"] = "link"
            state = self._write_state(state)
            return {
                "modelId": model_id,
                "linkId": link_id,
                "pointType": "link",
                "pointNo": result["point"].get("sequence"),
                "point": result["point"],
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

    def finish(self):
        with self._lock:
            state = self._require_state()
            model_id = state["modelId"]
            draft = self.store.get_draft(model_id)
            if state.get("captureMode") == "mixed_boundary":
                try:
                    draft = resolve_mixed_capture(draft)
                except MixedCaptureError as error:
                    raise ModelingSessionError(error.code, error.message)
                draft = self.store.save_draft(model_id, draft)
            links = list(draft.get("groupLinks") or [])
            incomplete_links = [link for link in links if len(link.get("points") or []) != 2]
            if incomplete_links:
                raise ModelingSessionError(
                    "MODELING_LINK_INCOMPLETE",
                    "each connection must contain its original start and end points",
                )

            groups = list(draft.get("groups") or [])
            incomplete_groups = [group for group in groups if len(group.get("points") or []) < 4]
            if incomplete_groups:
                raise ModelingSessionError(
                    "MODELING_AREA_INCOMPLETE",
                    "each modeling area requires at least four points",
                )

            for group in groups:
                recognized = self.store.recognize_group(model_id, group["id"])
                recognition = recognized.get("recognition") or {}
                if recognition.get("needsConfirmation") or not recognized.get("group", {}).get("subAreas"):
                    raise ModelingSessionError(
                        "MODELING_RECOGNITION_NEEDS_CONFIRMATION",
                        "the recorded area needs confirmation before a path can be generated",
                    )

            self.store.build_task_preview(model_id)
            generated = self.store.generate_task_plan(model_id)
            saved = self.store.save_model(model_id)
            state["status"] = "ready"
            state["currentPointType"] = None
            state = self._write_state(state)
            return {
                "modelId": model_id,
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
                "taskName": task_plan.get("taskName") or draft.get("name") or "",
                "updatedAt": draft.get("updatedAt") or task_plan.get("generatedAt"),
                "taskPreview": draft.get("taskPreview"),
                "taskPlan": task_plan,
            }
