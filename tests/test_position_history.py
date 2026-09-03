# coding=utf-8
from contextlib import contextmanager
import io
import json
import math
import os
import shutil
import tempfile
import unittest

from position_history import ModelingPositionHistory


@contextmanager
def temporary_directory():
    directory = tempfile.mkdtemp()
    try:
        yield directory
    finally:
        shutil.rmtree(directory, ignore_errors=True)


def write_json(path, payload):
    parent = os.path.dirname(path)
    if not os.path.exists(parent):
        os.makedirs(parent)
    with io.open(path, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=True))


def longitude_offset_cm(latitude, distance_cm):
    earth_radius_m = 6371000.0
    distance_m = float(distance_cm) / 100.0
    return math.degrees(distance_m / (earth_radius_m * math.cos(math.radians(latitude))))


class ModelingPositionHistoryTests(unittest.TestCase):
    def _write_model(self, root, model_id="model-1", points=None):
        write_json(os.path.join(root, "active_session.json"), {
            "status": "recording",
            "modelId": model_id,
        })
        write_json(os.path.join(root, "drafts", model_id + ".json"), {
            "id": model_id,
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "points": list(points or []),
            }],
            "groupLinks": [],
        })

    def test_fixed_rtk_uses_first_area_point_frame_and_builds_history(self):
        with temporary_directory() as root:
            lat = 32.0364
            lon = 118.9244
            self._write_model(root, points=[{
                "id": "p1", "lat": lat, "lon": lon,
            }])
            clock = [100.0]
            tracker = ModelingPositionHistory(
                root,
                now=lambda: clock[0],
                history_interval=0.2,
                minimum_distance_cm=3.0,
                flush_interval=0.0,
            )

            first = tracker.update(lat, lon, True, force_context=True)
            self.assertEqual({
                "x": 0,
                "y": 0,
                "coordinateReady": True,
                "rtkFixAvailable": True,
            }, first)

            clock[0] += 0.25
            second = tracker.update(
                lat,
                lon + longitude_offset_cm(lat, 10),
                True,
            )
            self.assertEqual(10, second["x"])
            self.assertEqual(0, second["y"])
            self.assertEqual([{"x": 0, "y": 0}, {"x": 10, "y": 0}], tracker.history()["points"])

    def test_missing_first_area_point_returns_not_ready_and_saves_nothing(self):
        with temporary_directory() as root:
            self._write_model(root, points=[])
            tracker = ModelingPositionHistory(root, now=lambda: 100.0, flush_interval=0.0)

            realtime = tracker.update(32.0, 118.0, True, force_context=True)

            self.assertEqual({
                "x": None,
                "y": None,
                "coordinateReady": False,
                "rtkFixAvailable": True,
            }, realtime)
            self.assertEqual([], tracker.history()["points"])

    def test_lost_rtk_does_not_append_or_discard_existing_history(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{
                "id": "p1", "lat": 32.0, "lon": 118.0,
            }])
            clock = [100.0]
            tracker = ModelingPositionHistory(root, now=lambda: clock[0], flush_interval=0.0)
            tracker.update(32.0, 118.0, True, force_context=True)

            clock[0] += 1.0
            realtime = tracker.update(32.1, 118.1, False)
            history = tracker.history()

            self.assertIsNone(realtime["x"])
            self.assertIsNone(realtime["y"])
            self.assertTrue(realtime["coordinateReady"])
            self.assertFalse(realtime["rtkFixAvailable"])
            self.assertEqual([{"x": 0, "y": 0}], history["points"])
            self.assertFalse(history["rtkFixAvailable"])

    def test_replacing_first_area_point_starts_a_new_coordinate_history(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{
                "id": "old-origin", "lat": 32.0, "lon": 118.0,
            }])
            clock = [100.0]
            tracker = ModelingPositionHistory(root, now=lambda: clock[0], flush_interval=0.0)
            tracker.update(32.0, 118.0, True, force_context=True)
            clock[0] += 1.0
            tracker.update(32.0, 118.0 + longitude_offset_cm(32.0, 10), True)
            self.assertEqual(2, len(tracker.history()["points"]))

            clock[0] += 1.0
            self._write_model(root, points=[{
                "id": "new-origin", "lat": 32.1, "lon": 118.1,
            }])
            tracker.update(32.1, 118.1, True, force_context=True)

            self.assertEqual([{"x": 0, "y": 0}], tracker.history()["points"])

    def test_history_survives_tracker_restart_for_same_model_frame(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{
                "id": "p1", "lat": 32.0, "lon": 118.0,
            }])
            first = ModelingPositionHistory(root, now=lambda: 100.0, flush_interval=0.0)
            first.update(32.0, 118.0, True, force_context=True)
            first.history()

            second = ModelingPositionHistory(root, now=lambda: 101.0, flush_interval=0.0)
            second.update(32.0, 118.0, True, force_context=True)

            self.assertEqual([{"x": 0, "y": 0}], second.history()["points"])


if __name__ == "__main__":
    unittest.main()

