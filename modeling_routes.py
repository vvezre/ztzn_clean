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
        return _error("MODEL_NOT_FOUND", "model not found", 404)
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

    return modeling_store
