# coding=utf-8
import json
import os
import re
import time
import uuid


MODEL_SCHEMA_VERSION = 1
MODEL_ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")


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
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)

    def _write_json(self, path, payload):
        tmp_path = path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        os.replace(tmp_path, path)

    def _new_model_id(self):
        return uuid.uuid4().hex[:12]

    def _normalize_model(self, payload, model_id=None, now=None, status=None):
        if not isinstance(payload, dict):
            raise InvalidModelPayloadError("model payload must be an object")

        current_time = self._timestamp(now)
        normalized = dict(payload)
        normalized["id"] = self._validate_model_id(model_id or normalized.get("id") or self._new_model_id())
        normalized["name"] = str(normalized.get("name") or "untitled-model")
        normalized["version"] = int(normalized.get("version") or MODEL_SCHEMA_VERSION)
        normalized["status"] = status or str(normalized.get("status") or "draft")
        normalized["createdAt"] = int(normalized.get("createdAt") or current_time)
        normalized["updatedAt"] = current_time
        normalized.setdefault("groups", [])
        normalized.setdefault("groupLinks", [])
        normalized.setdefault("recognition", {"confirmed": False, "items": []})
        normalized.setdefault("taskPreview", None)
        return normalized

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
