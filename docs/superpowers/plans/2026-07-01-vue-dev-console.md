# Vue Developer Console Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a read-only Vue developer console for RTK path-correction debugging.

**Architecture:** Add an independent Flask app under `dev_console` that reads Redis-like state and exposes `/dev/correction/state`. Serve a Vue 3 browser page that polls this endpoint and draws an RTK path preview on Canvas.

**Tech Stack:** Python, Flask, existing Redis/local Redis helpers, Vue 3 browser build, Canvas, unittest/pytest-compatible tests.

---

### Task 1: Correction State Builder

**Files:**
- Create: `dev_console/__init__.py`
- Create: `dev_console/correction_state.py`
- Test: `tests/test_dev_console_correction_state.py`

- [ ] **Step 1: Write failing tests**

Create tests for valid correction state, missing task fallback, and bounded trace storage.

- [ ] **Step 2: Run tests to verify failure**

Run: `python -m pytest tests/test_dev_console_correction_state.py -q`

Expected: FAIL because `dev_console.correction_state` does not exist.

- [ ] **Step 3: Implement minimal correction state builder**

Add pure helpers that read Redis-like objects, compute heading error, cross-track error, signed remaining, distance, and z-speed.

- [ ] **Step 4: Run tests to verify pass**

Run: `python -m pytest tests/test_dev_console_correction_state.py -q`

Expected: PASS.

### Task 2: Flask Dev Console App

**Files:**
- Create: `dev_console/app.py`
- Test: `tests/test_dev_console_app.py`

- [ ] **Step 1: Write failing endpoint tests**

Test that `/dev/correction/state` returns JSON and `/` serves HTML.

- [ ] **Step 2: Run tests to verify failure**

Run: `python -m pytest tests/test_dev_console_app.py -q`

Expected: FAIL because `dev_console.app` does not exist.

- [ ] **Step 3: Implement Flask app factory**

Create `create_app(redis_client=None, config_path='config.json')` and a CLI entrypoint.

- [ ] **Step 4: Run tests to verify pass**

Run: `python -m pytest tests/test_dev_console_app.py -q`

Expected: PASS.

### Task 3: Vue RTK Preview UI

**Files:**
- Create: `dev_console/static/index.html`

- [ ] **Step 1: Add Vue single-page UI**

Build a Canvas-based preview and a numeric data panel. Poll `/dev/correction/state` every 500 ms.

- [ ] **Step 2: Verify static route**

Run: `python -m pytest tests/test_dev_console_app.py -q`

Expected: PASS and HTML contains the Vue mount point.

### Task 4: Final Verification

**Files:**
- No new files.

- [ ] **Step 1: Run targeted test suite**

Run: `python -m pytest tests/test_dev_console_correction_state.py tests/test_dev_console_app.py -q`

Expected: PASS.

- [ ] **Step 2: Run syntax checks**

Run: `python -m py_compile dev_console/correction_state.py dev_console/app.py`

Expected: exit 0.
