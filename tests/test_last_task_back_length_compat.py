import os


ROOT = os.path.dirname(os.path.dirname(__file__))


def _read(name):
    with open(os.path.join(ROOT, name), "r", encoding="utf-8-sig") as handle:
        return handle.read()


def test_last_task_back_length_accepts_old_clients_without_field():
    main_text = _read("main.py")

    assert "data['lastTaskBackLength']" not in main_text
    assert 'data["lastTaskBackLength"]' not in main_text


def test_last_task_back_length_defaults_when_task_params_are_missing():
    main_text = _read("main.py")
    service_text = _read("service.py")

    unsafe = "int(taskParams.get('lastTaskBackLength'))"
    assert unsafe not in main_text
    assert unsafe not in service_text


def test_start_to_charging_distance_defaults_when_blank_or_missing():
    main_text = _read("main.py")
    service_text = _read("service.py")

    unsafe = "int(taskParams.get('startToChargingPilePointLength'))"
    assert unsafe not in main_text
    assert unsafe not in service_text


def test_missing_camera_does_not_create_unopened_capture_fallback():
    main_text = _read("main.py")

    assert "return cv2.VideoCapture(0)" not in main_text
