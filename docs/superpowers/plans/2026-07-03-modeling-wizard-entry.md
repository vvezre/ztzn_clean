# Modeling Wizard Entry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the frontend entry and first wizard screen for 打点建模: workbench entry, route registration, model list, new model, load draft, and next-step placeholder.

**Architecture:** Use the existing uni-app/Vue workbench grid pattern and the `api/modeling.js` wrappers from the previous feature. This feature is frontend-only and does not add RTK sampling, model recognition, planning, or execution.

**Tech Stack:** Vue 2, uni-app components, existing `request` wrapper, Node test runner static UI tests.

---

### Task 1: Add Static UI Test

**Files:**
- Create: `D:\vvezre\clean-frontend-github\tests\modelingWizardUi.test.js`

- [ ] Write a test that asserts:
  - `pages.json` registers `pages/work/modeling/index`.
  - `pages/work/index.vue` contains a `gotoModeling` method and `/pages/work/modeling/index`.
  - `pages/work/modeling/index.vue` imports model APIs.
  - The modeling page contains model list, create, refresh, save formal model, and next-step labels.

- [ ] Run `node --test tests/modelingWizardUi.test.js` and verify it fails because the page does not exist.

### Task 2: Register Page and Workbench Entry

**Files:**
- Modify: `D:\vvezre\clean-frontend-github\pages.json`
- Modify: `D:\vvezre\clean-frontend-github\pages\work\index.vue`

- [ ] Add `pages/work/modeling/index` to `pages.json`.
- [ ] Add one grid item in workbench labeled `打点建模`.
- [ ] Add `gotoModeling()` method that navigates to `/pages/work/modeling/index`.

### Task 3: Add Modeling Wizard First Screen

**Files:**
- Create: `D:\vvezre\clean-frontend-github\pages\work\modeling\index.vue`

- [ ] Build first screen with:
  - Header title `打点建模`.
  - Step indicators for `模型`, `区域组`, `打点`, `识别`, `连接`, `预览`, `执行`.
  - Model list loaded by `listModelingModels()`.
  - New model input and button using `createModelingModel(name)`.
  - Load draft using `getModelingDraft(modelId)`.
  - Manual save formal model using `saveModelingModel(modelId)`.
  - Auto-save draft method `autoSaveDraft()` that calls `saveModelingDraft(modelId, draft)`.
  - Disabled next button until a model is selected.
  - Placeholder text for the next step: `下一步：创建或选择区域组`.

### Task 4: Verify and Commit

- [ ] Run `node --test tests/modelingWizardUi.test.js`.
- [ ] Run `npm test`.
- [ ] Commit only the relevant frontend files.

