# coding=utf-8

import os

from flask import Flask, jsonify, request, send_from_directory

from dev_console.correction_state import build_correction_state
from dev_console.state_readers import (
    build_overview_state,
    build_redis_state,
    build_task_path_state,
    read_log_lines,
)
from local_redis import create_redis_client


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")


def create_app(redis_client=None, config_path="config.json", log_path="app.log"):
    app = Flask(__name__, static_folder=STATIC_DIR, static_url_path="/static")
    if hasattr(app, "json"):
        app.json.ensure_ascii = False
    else:
        app.config["JSON_AS_ASCII"] = False
    app.redis_client = redis_client or create_redis_client(
        host=os.environ.get("CLEANER_REDIS_HOST", "localhost"),
        port=int(os.environ.get("CLEANER_REDIS_PORT", "6379")),
        db=int(os.environ.get("CLEANER_REDIS_DB", "0")),
        decode_responses=True,
        local_mode=os.environ.get("CLEAN_LOCAL_MODE") == "1",
        fallback_if_unavailable=True,
    )
    app.dev_console_trace = []
    app.dev_console_config_path = config_path
    app.dev_console_log_path = log_path

    @app.route("/")
    def index():
        return send_from_directory(STATIC_DIR, "index.html")

    @app.route("/dev/correction/state")
    def correction_state():
        payload = build_correction_state(
            app.redis_client,
            config_path=app.dev_console_config_path,
            trace=app.dev_console_trace,
        )
        return jsonify(payload)

    @app.route("/dev/overview/state")
    def overview_state():
        return jsonify(build_overview_state(app.redis_client))

    @app.route("/dev/task-path")
    def task_path():
        return jsonify(build_task_path_state(app.redis_client, app.dev_console_config_path))

    @app.route("/dev/redis/state")
    def redis_state():
        return jsonify(build_redis_state(app.redis_client))

    @app.route("/dev/logs")
    def logs():
        query = request.args.get("query", "")
        limit = request.args.get("limit", "200")
        return jsonify(read_log_lines(app.dev_console_log_path, query=query, limit=limit))

    return app


def main():
    port = int(os.environ.get("CLEANER_DEV_CONSOLE_PORT", "7900"))
    app = create_app(
        config_path=os.environ.get("CLEANER_TASK_CONFIG", "config.json"),
        log_path=os.environ.get("CLEANER_APP_LOG", "app.log"),
    )
    app.run(host="0.0.0.0", port=port, debug=False)


if __name__ == "__main__":
    main()
