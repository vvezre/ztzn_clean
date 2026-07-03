# Modeling Store Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first modeling feature: local JSON model management plus auto-saved drafts, exposed through backend routes and frontend API wrappers.

**Architecture:** Add a focused local JSON store for modeling data, then expose it through `/modeling/*` Flask routes. Keep this feature independent from RTK sampling, recognition, planning, and execution; those later features will consume the persisted model/draft contract.

**Tech Stack:** Python `unittest`, Flask routes in `main.py`, JSON files on disk, existing Vue/uni-app request wrapper.

---

## File Structure

- Create `D:\vvezre\clean_fsm\modeling_store.py`
  - Owns local JSON persistence for formal models and drafts.
  - Generates safe model IDs.
  - Enforces model JSON envelope fields.
  - Has no Flask dependency.
- Create `D:\vvezre\clean_fsm\modeling_routes.py`
  - Registers `/modeling/*` routes on an existing Flask app.
  - Converts store results to JSON responses.
- Modify `D:\vvezre\clean_fsm\main.py`
  - Registers modeling routes on startup.
  - Uses `MODELING_STORE_DIR` env var or `modeling_models` folder by default.
- Create `D:\vvezre\clean_fsm\tests\test_modeling_store.py`
  - Unit tests for local JSON model/draft persistence.
- Create `D:\vvezre\clean_fsm\tests\test_modeling_routes.py`
  - Flask route tests using a temporary model directory.
- Create `D:\vvezre\clean_fsm\tests\test_modeling_main_routes.py`
  - Source-level test that `main.py` wires routes.
- Create `D:\vvezre\clean-frontend-github\api\modeling.js`
  - Frontend API wrappers for model list, create, load, save, delete, draft save/load.
- Create `D:\vvezre\clean-frontend-github\tests\modelingApi.test.js`
  - Static tests that API wrappers call the correct endpoints.

---

### Task 1: Add Local JSON Modeling Store

**Files:**
- Create: `D:\vvezre\clean_fsm\modeling_store.py`
- Test: `D:\vvezre\clean_fsm\tests\test_modeling_store.py`

- [ ] **Step 1: Write failing store tests**

Create `D:\vvezre\clean_fsm\tests\test_modeling_store.py`:

```python
import os
import tempfile
import unittest


class ModelingStoreTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir)

    def test_create_model_writes_formal_model_and_empty_draft(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)

        model = store.create_model("现场模型A")

        self.assertEqual(model["name"], "现场模型A")
        self.assertEqual(model["status"], "draft")
        self.assertEqual(model["version"], 1)
        self.assertEqual(model["groups"], [])
        self.assertTrue(os.path.exists(os.path.join(self.tmpdir, "models", model["id"] + ".json")))
        self.assertTrue(os.path.exists(os.path.join(self.tmpdir, "drafts", model["id"] + ".json")))

    def test_save_draft_updates_draft_without_marking_formal_model_saved(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("模型草稿")
        draft = dict(model)
        draft["groups"] = [{"id": "g1", "name": "区域组1", "points": []}]

        saved = store.save_draft(model["id"], draft, now=1005)
        formal = store.get_model(model["id"])

        self.assertEqual(saved["groups"][0]["id"], "g1")
        self.assertEqual(saved["status"], "draft")
        self.assertEqual(saved["updatedAt"], 1005)
        self.assertEqual(formal["groups"], [])

    def test_save_model_promotes_draft_to_formal_model(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("正式模型")
        draft = dict(model)
        draft["groups"] = [{"id": "g1", "name": "区域组1", "points": []}]
        store.save_draft(model["id"], draft, now=1005)

        saved = store.save_model(model["id"], now=1010)

        self.assertEqual(saved["status"], "saved")
        self.assertEqual(saved["savedAt"], 1010)
        self.assertEqual(store.get_model(model["id"])["groups"][0]["id"], "g1")

    def test_list_models_returns_saved_metadata_without_large_body(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("列表模型")
        store.save_model(model["id"], now=1010)

        items = store.list_models()

        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["id"], model["id"])
        self.assertEqual(items[0]["name"], "列表模型")
        self.assertEqual(items[0]["status"], "saved")
        self.assertNotIn("groups", items[0])

    def test_delete_model_removes_model_and_draft_files(self):
        from modeling_store import ModelingStore, ModelNotFoundError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("删除模型")

        removed = store.delete_model(model["id"])

        self.assertTrue(removed)
        with self.assertRaises(ModelNotFoundError):
            store.get_model(model["id"])
        self.assertFalse(os.path.exists(os.path.join(self.tmpdir, "drafts", model["id"] + ".json")))

    def test_rejects_unsafe_model_id(self):
        from modeling_store import ModelingStore, InvalidModelIdError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)

        with self.assertRaises(InvalidModelIdError):
            store.get_model("../bad")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests and verify they fail**

Run:

```powershell
python -m unittest tests.test_modeling_store
```

Expected:

```text
ModuleNotFoundError: No module named 'modeling_store'
```

- [ ] **Step 3: Implement `modeling_store.py`**

Create `D:\vvezre\clean_fsm\modeling_store.py`:

```python
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
        with open(path, "r") as handle:
            return json.load(handle)

    def _write_json(self, path, payload):
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as handle:
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
        normalized["name"] = str(normalized.get("name") or "未命名模型")
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
```

- [ ] **Step 4: Run store tests and verify they pass**

Run:

```powershell
python -m unittest tests.test_modeling_store
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit store unit**

Run:

```powershell
git add modeling_store.py tests/test_modeling_store.py
git commit -m "Add modeling JSON store"
```

---

### Task 2: Add Backend Modeling Routes

**Files:**
- Create: `D:\vvezre\clean_fsm\modeling_routes.py`
- Test: `D:\vvezre\clean_fsm\tests\test_modeling_routes.py`

- [ ] **Step 1: Write failing route tests**

Create `D:\vvezre\clean_fsm\tests\test_modeling_routes.py`:

```python
import tempfile
import unittest
from flask import Flask


class ModelingRoutesTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        app = Flask(__name__)
        from modeling_routes import register_modeling_routes
        register_modeling_routes(app, storage_dir=self.tmpdir)
        self.client = app.test_client()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir)

    def test_create_list_load_save_and_delete_model(self):
        created = self.client.post("/modeling/models", json={"name": "现场A"})
        self.assertEqual(created.status_code, 200)
        model = created.get_json()["data"]
        self.assertEqual(model["name"], "现场A")

        listed = self.client.get("/modeling/models")
        self.assertEqual(listed.status_code, 200)
        self.assertEqual(listed.get_json()["data"][0]["id"], model["id"])

        loaded = self.client.get("/modeling/models/" + model["id"])
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.get_json()["data"]["id"], model["id"])

        saved = self.client.post("/modeling/models/{}/save".format(model["id"]))
        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["status"], "saved")

        deleted = self.client.delete("/modeling/models/" + model["id"])
        self.assertEqual(deleted.status_code, 200)
        self.assertTrue(deleted.get_json()["success"])

    def test_save_and_read_draft(self):
        created = self.client.post("/modeling/models", json={"name": "草稿A"}).get_json()["data"]
        draft = dict(created)
        draft["groups"] = [{"id": "g1", "name": "区域组1", "points": []}]

        saved = self.client.post("/modeling/draft", json={"modelId": created["id"], "draft": draft})
        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["groups"][0]["id"], "g1")

        loaded = self.client.get("/modeling/draft/" + created["id"])
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.get_json()["data"]["groups"][0]["name"], "区域组1")

    def test_missing_model_returns_404_response(self):
        response = self.client.get("/modeling/models/missing")

        self.assertEqual(response.status_code, 404)
        self.assertFalse(response.get_json()["success"])
        self.assertEqual(response.get_json()["code"], "MODEL_NOT_FOUND")

    def test_invalid_payload_returns_400_response(self):
        response = self.client.post("/modeling/draft", json={"modelId": "", "draft": []})

        self.assertEqual(response.status_code, 400)
        self.assertFalse(response.get_json()["success"])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run route tests and verify they fail**

Run:

```powershell
python -m unittest tests.test_modeling_routes
```

Expected:

```text
ModuleNotFoundError: No module named 'modeling_routes'
```

- [ ] **Step 3: Implement `modeling_routes.py`**

Create `D:\vvezre\clean_fsm\modeling_routes.py`:

```python
# coding=utf-8
from flask import jsonify, request

from modeling_store import (
    InvalidModelIdError,
    InvalidModelPayloadError,
    ModelNotFoundError,
    ModelingStore,
)


def _ok(data=None, msg="ok"):
    payload = {
        "success": True,
        "code": "OK",
        "msg": msg,
    }
    if data is not None:
        payload["data"] = data
    return jsonify(payload)


def _error(code, msg, status_code):
    return jsonify({
        "success": False,
        "code": code,
        "msg": msg,
    }), status_code


def _handle_store_error(error):
    if isinstance(error, ModelNotFoundError):
        return _error("MODEL_NOT_FOUND", "模型不存在", 404)
    if isinstance(error, (InvalidModelIdError, InvalidModelPayloadError)):
        return _error("INVALID_MODEL_PAYLOAD", str(error), 400)
    return _error("MODELING_STORE_ERROR", str(error), 500)


def register_modeling_routes(app, storage_dir=None, store=None):
    modeling_store = store or ModelingStore(storage_dir or "modeling_models")

    @app.route("/modeling/models", methods=["GET"])
    def modeling_list_models():
        try:
            return _ok(modeling_store.list_models())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/models", methods=["POST"])
    def modeling_create_model():
        payload = request.get_json(silent=True) or {}
        name = payload.get("name") or "未命名模型"
        try:
            return _ok(modeling_store.create_model(name))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/models/<model_id>", methods=["GET"])
    def modeling_get_model(model_id):
        try:
            return _ok(modeling_store.get_model(model_id))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/models/<model_id>/save", methods=["POST"])
    def modeling_save_model(model_id):
        try:
            return _ok(modeling_store.save_model(model_id))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/models/<model_id>", methods=["DELETE"])
    def modeling_delete_model(model_id):
        try:
            modeling_store.delete_model(model_id)
            return _ok({"id": model_id}, msg="deleted")
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/draft/<model_id>", methods=["GET"])
    def modeling_get_draft(model_id):
        try:
            return _ok(modeling_store.get_draft(model_id))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/draft", methods=["POST"])
    def modeling_save_draft():
        payload = request.get_json(silent=True) or {}
        model_id = payload.get("modelId")
        draft = payload.get("draft")
        try:
            return _ok(modeling_store.save_draft(model_id, draft))
        except Exception as error:
            return _handle_store_error(error)

    return modeling_store
```

- [ ] **Step 4: Run store and route tests**

Run:

```powershell
python -m unittest tests.test_modeling_store tests.test_modeling_routes
```

Expected:

```text
OK
```

- [ ] **Step 5: Commit route unit**

Run:

```powershell
git add modeling_routes.py tests/test_modeling_routes.py
git commit -m "Add modeling model routes"
```

---

### Task 3: Wire Modeling Routes Into Main App

**Files:**
- Modify: `D:\vvezre\clean_fsm\main.py`
- Test: `D:\vvezre\clean_fsm\tests\test_modeling_main_routes.py`

- [ ] **Step 1: Write failing source test**

Create `D:\vvezre\clean_fsm\tests\test_modeling_main_routes.py`:

```python
import io
import os
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_PATH = os.path.join(ROOT, "main.py")


class ModelingMainRoutesTest(unittest.TestCase):
    def test_main_registers_modeling_routes(self):
        with io.open(MAIN_PATH, "r", encoding="utf-8") as handle:
            source = handle.read()

        self.assertIn("from modeling_routes import register_modeling_routes", source)
        self.assertIn("MODELING_STORE_DIR", source)
        self.assertIn("register_modeling_routes(app", source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test and verify it fails**

Run:

```powershell
python -m unittest tests.test_modeling_main_routes
```

Expected:

```text
FAIL: test_main_registers_modeling_routes
```

- [ ] **Step 3: Modify `main.py` imports**

Add near other local imports in `D:\vvezre\clean_fsm\main.py`:

```python
from modeling_routes import register_modeling_routes
```

- [ ] **Step 4: Modify `main.py` route registration**

Add after existing dev route definitions and before `/vehicle/login`:

```python
MODELING_STORE_DIR = os.environ.get("MODELING_STORE_DIR", os.path.join(os.getcwd(), "modeling_models"))
register_modeling_routes(app, storage_dir=MODELING_STORE_DIR)
```

- [ ] **Step 5: Run route wiring test**

Run:

```powershell
python -m unittest tests.test_modeling_main_routes
```

Expected:

```text
OK
```

- [ ] **Step 6: Commit main wiring**

Run:

```powershell
git add main.py tests/test_modeling_main_routes.py
git commit -m "Wire modeling routes into main app"
```

---

### Task 4: Add Frontend Modeling API Wrappers

**Files:**
- Create: `D:\vvezre\clean-frontend-github\api\modeling.js`
- Test: `D:\vvezre\clean-frontend-github\tests\modelingApi.test.js`

- [ ] **Step 1: Write failing frontend API test**

Create `D:\vvezre\clean-frontend-github\tests\modelingApi.test.js`:

```javascript
const assert = require('node:assert/strict')
const fs = require('node:fs')
const path = require('node:path')
const test = require('node:test')

const root = path.resolve(__dirname, '..')

function read(file) {
  return fs.readFileSync(path.join(root, file), 'utf8')
}

test('modeling APIs are exported with expected endpoints', () => {
  const source = read('api/modeling.js')

  assert.match(source, /export function listModelingModels/)
  assert.match(source, /\/modeling\/models/)
  assert.match(source, /export function createModelingModel/)
  assert.match(source, /export function getModelingModel/)
  assert.match(source, /export function saveModelingModel/)
  assert.match(source, /\/modeling\/models\/\$\\{encodeURIComponent\\(modelId\\)\\}\/save/)
  assert.match(source, /export function deleteModelingModel/)
  assert.match(source, /export function getModelingDraft/)
  assert.match(source, /\/modeling\/draft\/\$\\{encodeURIComponent\\(modelId\\)\\}/)
  assert.match(source, /export function saveModelingDraft/)
})
```

- [ ] **Step 2: Run frontend test and verify it fails**

Run:

```powershell
cd D:\vvezre\clean-frontend-github
node --test tests/modelingApi.test.js
```

Expected:

```text
ENOENT: no such file or directory, open '...\api\modeling.js'
```

- [ ] **Step 3: Implement `api/modeling.js`**

Create `D:\vvezre\clean-frontend-github\api\modeling.js`:

```javascript
import request from '@/utils/request'

export function listModelingModels() {
  return request({
    url: '/modeling/models',
    method: 'get',
    allowBusinessResponse: true
  })
}

export function createModelingModel(name) {
  return request({
    url: '/modeling/models',
    method: 'post',
    data: { name },
    allowBusinessResponse: true
  })
}

export function getModelingModel(modelId) {
  return request({
    url: `/modeling/models/${encodeURIComponent(modelId)}`,
    method: 'get',
    allowBusinessResponse: true
  })
}

export function saveModelingModel(modelId) {
  return request({
    url: `/modeling/models/${encodeURIComponent(modelId)}/save`,
    method: 'post',
    allowBusinessResponse: true
  })
}

export function deleteModelingModel(modelId) {
  return request({
    url: `/modeling/models/${encodeURIComponent(modelId)}`,
    method: 'delete',
    allowBusinessResponse: true
  })
}

export function getModelingDraft(modelId) {
  return request({
    url: `/modeling/draft/${encodeURIComponent(modelId)}`,
    method: 'get',
    allowBusinessResponse: true
  })
}

export function saveModelingDraft(modelId, draft) {
  return request({
    url: '/modeling/draft',
    method: 'post',
    data: {
      modelId,
      draft
    },
    allowBusinessResponse: true
  })
}
```

- [ ] **Step 4: Run frontend API test**

Run:

```powershell
cd D:\vvezre\clean-frontend-github
node --test tests/modelingApi.test.js
```

Expected:

```text
pass
```

- [ ] **Step 5: Commit frontend API wrappers**

Run:

```powershell
git -C D:\vvezre\clean-frontend-github add api/modeling.js tests/modelingApi.test.js
git -C D:\vvezre\clean-frontend-github commit -m "Add modeling API wrappers"
```

---

### Task 5: Full Feature Verification

**Files:**
- Verify backend and frontend files from Tasks 1-4.

- [ ] **Step 1: Run backend modeling tests**

Run:

```powershell
cd D:\vvezre\clean_fsm
python -m unittest tests.test_modeling_store tests.test_modeling_routes tests.test_modeling_main_routes
```

Expected:

```text
OK
```

- [ ] **Step 2: Run backend runtime regression tests touched by route registration**

Run:

```powershell
cd D:\vvezre\clean_fsm
python -m unittest tests.test_runtime_state tests.test_dev_console_main_routes
```

Expected:

```text
OK
```

- [ ] **Step 3: Run frontend modeling API test**

Run:

```powershell
cd D:\vvezre\clean-frontend-github
node --test tests/modelingApi.test.js
```

Expected:

```text
pass
```

- [ ] **Step 4: Confirm no unrelated files are staged**

Run:

```powershell
git -C D:\vvezre\clean_fsm diff --cached --name-only
git -C D:\vvezre\clean-frontend-github diff --cached --name-only
```

Expected:

```text

```

- [ ] **Step 5: Record feature completion**

Reply with:

```text
模型管理与草稿保存已完成：后端本地 JSON store、/modeling 模型接口、前端 API wrapper 均已通过测试。下一功能建议进入“工作台打点建模入口 + 模型选择向导首页”。
```

---

## Self-Review Notes

Spec coverage for this first feature:

- Covered: model list new/save/load/delete, local JSON storage, auto-save draft contract, formal manual save contract, backend API, frontend API wrappers.
- Not covered in this feature: RTK sampling, region group UI, automatic recognition, route planning, execution, developer debug panel. These are intentionally deferred to later feature plans.

Placeholder scan:

- No `TBD`, `TODO`, or unspecified implementation steps.
- Every code-changing step includes exact file path and concrete code.

Type consistency:

- Backend uses `modelId` in route JSON payloads and `model_id` in Python internals.
- Frontend wrapper names consistently use `ModelingModel` and `ModelingDraft`.
