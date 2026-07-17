# Multi-Waypoint Loop Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add optional finite-count and continuous closed-loop execution to the existing RTK multi-waypoint navigation flow.

**Architecture:** Keep waypoint CRUD unchanged. Parse loop options at the start endpoint, persist runtime options and progress in Redis, and let the existing navigation thread execute either one linear pass or repeated closed cycles. Extend the existing UniApp panel and API action to submit loop options and render point/cycle progress.

**Tech Stack:** Python, Flask, Redis, pytest/unittest, Vue 2, UniApp, Node.js tests.

---

### Task 1: Backend Loop Contract

**Files:**
- Modify: `tests/test_go_to_point_integration.py`
- Modify: `main.py`

- [ ] Add failing integration assertions for start payload parsing, loop validation, loop progress fields, and the closed route sequence.
- [ ] Run `python -m pytest tests/test_go_to_point_integration.py -q` and verify the new assertions fail because loop support is absent.
- [ ] Add small loop-option helpers that normalize `loop`, `loopMode`, and `loopCount`.
- [ ] Store `waypointLoopEnabled`, `waypointLoopMode`, `waypointLoopTarget`, and `waypointLoopCurrent` before starting the navigation thread.
- [ ] Reject closed-loop startup with fewer than two waypoints or an invalid finite loop count.
- [ ] Extend `/vehicle/goToPoints/progress` with loop mode and cycle progress.

### Task 2: Backend Closed-Loop Execution

**Files:**
- Modify: `tests/test_go_to_point_integration.py`
- Modify: `main.py`

- [ ] Add failing assertions that the thread first reaches `P1`, then runs `P2...Pn...P1`, increments the cycle only after returning to `P1`, and stops at the target count.
- [ ] Run the focused test and verify failure.
- [ ] Extract the existing per-target movement into a local helper so single-pass and loop modes share turn-and-drive behavior.
- [ ] Preserve single-pass behavior as `P1 -> ... -> Pn`.
- [ ] Implement closed-loop behavior as initial approach to `P1`, followed by repeated `P2 -> ... -> Pn -> P1`.
- [ ] Stop immediately on parking, manual stop, turn failure, drive failure, or completion of the configured count.
- [ ] Reset runtime loop state in the thread cleanup without deleting stored waypoints.
- [ ] Run backend focused and regression tests.

### Task 3: Frontend API and State

**Files:**
- Modify: `clean-frontend-github/tests/loopAutoCleanUi.test.js`
- Modify: `clean-frontend-github/api/login.js`
- Modify: `clean-frontend-github/store/modules/user.js`
- Modify: `clean-frontend-github/pages/index.vue`

- [ ] Add failing source-level tests for submitting loop options and displaying current/target cycle state.
- [ ] Run `node tests/loopAutoCleanUi.test.js` and verify the new checks fail.
- [ ] Change `startMultiGoToPoint(options)` to POST the selected options.
- [ ] Pass options through the Vuex `StartMultiGoToPoint` action.
- [ ] Extend `waypoints` state with execution mode, loop mode, target count, and current cycle.
- [ ] Map progress response fields into the page state.

### Task 4: Frontend Controls

**Files:**
- Modify: `clean-frontend-github/pages/index.vue`

- [ ] Add single/closed-loop execution controls.
- [ ] Add finite/continuous controls and a numeric cycle input visible only for finite closed-loop mode.
- [ ] Show the route summary with a return arrow to `P1` in closed-loop mode.
- [ ] Show current target point and current/target cycle while running.
- [ ] Disable waypoint editing and configuration while navigation is active.
- [ ] Remove the manual latitude/longitude add row.
- [ ] Submit validated loop options when starting navigation.

### Task 5: Verification

**Files:**
- Verify: `clean/main.py`
- Verify: `clean-frontend-github/pages/index.vue`
- Build: `clean-frontend-github/dist/build/h5`

- [ ] Run backend waypoint and point-to-point tests.
- [ ] Run Python compilation checks.
- [ ] Run frontend source tests.
- [ ] Run the H5 production build.
- [ ] Inspect the final diff for unrelated changes and whitespace errors.
