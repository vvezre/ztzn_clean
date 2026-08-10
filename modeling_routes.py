# coding=utf-8
from flask import jsonify, request

from modeling_execution import ModelingExecutionError
from modeling_preview import ModelingPreviewError
from modeling_task_generator import ModelingTaskGenerationError
from modeling_store import (
    InvalidModelIdError,
    InvalidModelPayloadError,
    ModelNotFoundError,
    ModelingStore,
)
from modeling_sampler import ModelingSampleError, inspect_sample_readiness
from modeling_session import ModelingSession, ModelingSessionError


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
    if isinstance(error, ModelingSessionError):
        return _error(error.code, error.message, 400)
    if isinstance(error, ModelingSampleError):
        return _error(error.code, error.message, 400)
    if isinstance(error, ModelingPreviewError):
        return _error("MODELING_PREVIEW_ERROR", str(error), 400)
    if isinstance(error, ModelingTaskGenerationError):
        return _error("MODELING_TASK_GENERATION_ERROR", str(error), 400)
    if isinstance(error, ModelingExecutionError):
        return _error("MODELING_EXECUTION_ERROR", str(error), 400)
    if isinstance(error, ModelNotFoundError):
        return _error("MODEL_NOT_FOUND", "model not found", 404)
    if isinstance(error, (InvalidModelIdError, InvalidModelPayloadError)):
        return _error("INVALID_MODEL_PAYLOAD", str(error), 400)
    if getattr(error, "code", None) and getattr(error, "message", None):
        status_code = 409 if error.code == "TASK_NAME_EXISTS" else 400
        return _error(error.code, error.message, status_code)
    return _error("MODELING_STORE_ERROR", str(error), 500)


def _request_model_id(payload=None):
    payload = payload or {}
    return payload.get("modelId") or request.args.get("modelId")


def register_modeling_routes(app, storage_dir=None, store=None, sample_point_provider=None,
                             task_execution_starter=None, task_progress_reader=None,
                             task_stop_handler=None, task_save_handler=None):
    modeling_store = store or ModelingStore(storage_dir or "modeling_models")
    modeling_session = ModelingSession(modeling_store, sample_point_provider)

    @app.route("/modeling/session/start", methods=["POST"])
    def modeling_session_start():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_session.start(
                name=payload.get("name"),
                restart=bool(payload.get("restart", False)),
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/current", methods=["GET"])
    def modeling_session_current():
        try:
            return _ok(modeling_session.current())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/path", methods=["GET"])
    def modeling_session_path():
        try:
            return _ok(modeling_session.current_path())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/save-task", methods=["POST"])
    def modeling_session_save_task():
        payload = request.get_json(silent=True) or {}
        try:
            if task_save_handler is None:
                raise ModelingSessionError(
                    "MODELING_TASK_SAVE_NOT_CONFIGURED",
                    "modeling task save handler is not configured",
                )
            if modeling_session.current().get("status") != "ready":
                raise ModelingSessionError(
                    "MODELING_PATH_NOT_READY",
                    "finish modeling before saving the route",
                )
            current_path = modeling_session.current_path()
            return _ok(
                task_save_handler(payload.get("taskName"), current_path),
                msg="saved",
            )
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/record-area-point", methods=["POST"])
    def modeling_session_record_area_point():
        try:
            return _ok(modeling_session.record_area_point())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/record-link-point", methods=["POST"])
    def modeling_session_record_link_point():
        try:
            return _ok(modeling_session.record_link_point())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/new-area", methods=["POST"])
    def modeling_session_new_area():
        try:
            return _ok(modeling_session.new_area())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/undo", methods=["POST"])
    def modeling_session_undo():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_session.undo(payload.get("pointType")))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/delete-area-point", methods=["POST"])
    def modeling_session_delete_area_point():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(
                modeling_session.delete_area_point(payload.get("id")),
                msg="deleted",
            )
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/delete-link-point", methods=["POST"])
    def modeling_session_delete_link_point():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(
                modeling_session.delete_link_point(payload.get("id")),
                msg="deleted",
            )
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/clear", methods=["POST"])
    def modeling_session_clear():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_session.clear(payload.get("pointType")))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/clear-all", methods=["POST"])
    def modeling_session_clear_all():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_session.clear_all(payload.get("pointType")))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/finish", methods=["POST"])
    def modeling_session_finish():
        try:
            return _ok(modeling_session.finish())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/session/replan", methods=["POST"])
    def modeling_session_replan():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_session.replan(payload.get("areaOrder")))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/models", methods=["GET"])
    def modeling_list_models():
        try:
            return _ok(modeling_store.list_models())
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/models", methods=["POST"])
    def modeling_create_model():
        payload = request.get_json(silent=True) or {}
        name = payload.get("name") or "untitled-model"
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

    @app.route("/modeling/preview", methods=["POST"])
    def modeling_build_preview():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.build_task_preview(_request_model_id(payload)))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/tasks/generate", methods=["POST"])
    def modeling_generate_tasks():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.generate_task_plan(_request_model_id(payload)))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/tasks/start", methods=["POST"])
    def modeling_start_tasks():
        payload = request.get_json(silent=True) or {}
        try:
            if task_execution_starter is None:
                raise ModelingExecutionError("modeling task executor is not configured")
            model_id = _request_model_id(payload)
            draft = modeling_store.get_draft(model_id)
            return _ok(task_execution_starter(model_id, draft, payload))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/tasks/progress", methods=["GET"])
    def modeling_task_progress():
        payload = request.args.to_dict()
        try:
            if task_progress_reader is None:
                raise ModelingExecutionError("modeling task progress reader is not configured")
            payload["modelId"] = _request_model_id(payload)
            return _ok(task_progress_reader(payload))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/tasks/stop", methods=["POST"])
    def modeling_stop_tasks():
        payload = request.get_json(silent=True) or {}
        try:
            if task_stop_handler is None:
                raise ModelingExecutionError("modeling task stop handler is not configured")
            payload["modelId"] = _request_model_id(payload)
            return _ok(task_stop_handler(payload))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/sample-status", methods=["GET"])
    def modeling_sample_status():
        try:
            return _ok(inspect_sample_readiness(sample_point_provider))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups", methods=["POST"])
    def modeling_create_group():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.create_group(
                payload.get("modelId"),
                payload.get("name"),
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/group-links", methods=["POST"])
    def modeling_create_group_link():
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.create_group_link(
                _request_model_id(payload),
                payload,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/group-links/<link_id>/sample-point", methods=["POST"])
    def modeling_sample_group_link_point(link_id):
        payload = request.get_json(silent=True) or {}
        try:
            if sample_point_provider is None:
                raise ModelingSampleError("RTK_SAMPLE_PROVIDER_MISSING", "sample provider is missing")
            point = sample_point_provider()
            return _ok(modeling_store.append_group_link_point(
                _request_model_id(payload),
                link_id,
                point,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/group-links/<link_id>", methods=["DELETE"])
    def modeling_delete_group_link(link_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.delete_group_link(
                _request_model_id(payload),
                link_id,
            ), msg="deleted")
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>", methods=["PATCH"])
    def modeling_update_group(group_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.update_group(
                _request_model_id(payload),
                group_id,
                payload,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>/clear-points", methods=["POST"])
    def modeling_clear_group_points(group_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.clear_group_points(
                _request_model_id(payload),
                group_id,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>/sample-point", methods=["POST"])
    def modeling_sample_group_point(group_id):
        payload = request.get_json(silent=True) or {}
        try:
            if sample_point_provider is None:
                raise ModelingSampleError("RTK_SAMPLE_PROVIDER_MISSING", "sample provider is missing")
            point = sample_point_provider()
            return _ok(modeling_store.append_group_point(
                _request_model_id(payload),
                group_id,
                point,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>/points/<point_id>", methods=["DELETE"])
    def modeling_delete_group_point(group_id, point_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.delete_group_point(
                _request_model_id(payload),
                group_id,
                point_id,
            ), msg="deleted")
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>/recognize", methods=["POST"])
    def modeling_recognize_group(group_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.recognize_group(
                _request_model_id(payload),
                group_id,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>/confirm-recognition", methods=["POST"])
    def modeling_confirm_group_recognition(group_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.confirm_group_recognition(
                _request_model_id(payload),
                group_id,
            ))
        except Exception as error:
            return _handle_store_error(error)

    @app.route("/modeling/groups/<group_id>", methods=["DELETE"])
    def modeling_delete_group(group_id):
        payload = request.get_json(silent=True) or {}
        try:
            return _ok(modeling_store.delete_group(
                _request_model_id(payload),
                group_id,
            ), msg="deleted")
        except Exception as error:
            return _handle_store_error(error)

    return modeling_store
