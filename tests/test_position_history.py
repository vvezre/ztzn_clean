# coding=utf-8
from contextlib import contextmanager
import io
import json
import math
import os
import shutil
import tempfile
import unittest

from position_history import ModelingPositionHistory, history_capture_metadata


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
            "currentGroupId": "g1",
            "captureMode": None,
            "currentPointType": "area",
        })
        write_json(os.path.join(root, "drafts", model_id + ".json"), {
            "id": model_id,
            "groups": [{
                "id": "g1",
                "areaNumber": 1,
                "points": list(points or []),
            }, {"id": "g2", "areaNumber": 2, "points": []},
                {"id": "g3", "areaNumber": 3, "points": []}],
            "groupLinks": [{"id": "bridge1", "linkNumber": 1, "points": []}],
        })

    def _switch_capture(self, root, area=1, link=None, status="recording"):
        # Atomic replacement also mirrors ModelingSession._write_state.
        path = os.path.join(root, "active_session.json")
        with io.open(path, "r", encoding="utf-8") as handle:
            state = json.load(handle)
        state.update({"currentGroupId": "g{}".format(area),
                      "currentLinkId": None, "pendingLinkNumber": link,
                      "captureMode": "link" if link else None,
                      "currentPointType": "link" if link else "area", "status": status})
        write_json(path + ".tmp", state)
        os.replace(path + ".tmp", path)

    def test_area_queries_exclude_bridges_and_keep_one_global_frame(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{"id": "p1", "lat": 32.0, "lon": 118.0}])
            clock = [100.0]
            tracker = ModelingPositionHistory(root, now=lambda: clock[0], flush_interval=0)

            def sample(x):
                tracker.update(32.0, 118.0 + longitude_offset_cm(32.0, x), True, force_context=True)
                clock[0] += 1

            sample(0)
            sample(10)
            self._switch_capture(root, link=1)
            sample(20)  # Includes the interval before the first bridge point.
            self._switch_capture(root, area=2)
            sample(30)
            sample(40)
            self._switch_capture(root, area=2, link=2)
            sample(50)
            self._switch_capture(root, area=3)
            sample(60)
            self._switch_capture(root, area=3, status="ready")
            sample(70)  # Preview/execution positions are not area-recording points.
            self.assertEqual([{"x": x, "y": 0} for x in (0, 10)], tracker.history(1)["points"])
            self.assertEqual([{"x": x, "y": 0} for x in (30, 40)], tracker.history(2)["points"])
            self.assertEqual([{"x": 60, "y": 0}], tracker.history(3)["points"])
            self.assertEqual([], tracker.history(4)["points"])
            self.assertEqual([{"x": x, "y": 0} for x in range(0, 80, 10)], tracker.history()["points"])
            restarted = ModelingPositionHistory(root, now=lambda: clock[0])
            restarted.update(32, 118, False, force_context=True)
            for area in (1, 2, 3):
                self.assertEqual(tracker.history(area)["points"], restarted.history(area)["points"])

    def test_area_switch_obeys_global_interval_but_keeps_same_position_in_new_area(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{"id": "p1", "lat": 32, "lon": 118}])
            clock = [100.0]
            tracker = ModelingPositionHistory(root, now=lambda: clock[0])
            tracker.update(32, 118, True, force_context=True)
            self._switch_capture(root, area=2)
            clock[0] += 0.9
            tracker.update(32, 118, True, force_context=True)
            self.assertEqual([], tracker.history(2)["points"])
            clock[0] += 0.1
            tracker.update(32, 118, True, force_context=True)
            self.assertEqual([{"x": 0, "y": 0}], tracker.history(2)["points"])
            clock[0] += 1
            tracker.update(32, 118 + longitude_offset_cm(32, 2), True)
            self.assertEqual(2, len(tracker.history()["points"]))
            tracker.update(32, 118 + longitude_offset_cm(32, 10), False)
            self.assertEqual(1, len(tracker.history(2)["points"]))
            self.assertFalse(tracker.history(2)["rtkFixAvailable"])

    def test_history_cap_is_shared_across_all_areas(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{"id": "p1", "lat": 32, "lon": 118}])
            clock = [100.0]
            tracker = ModelingPositionHistory(root, now=lambda: clock[0], max_points=3)
            for index in range(5):
                self._switch_capture(root, area=1 if index < 3 else 2)
                tracker.update(32, 118 + longitude_offset_cm(32, index * 10), True, force_context=True)
                clock[0] += 1
            self.assertEqual(3, len(tracker.history()["points"]))
            self.assertEqual([{"x": 20, "y": 0}], tracker.history(1)["points"])
            self.assertEqual([{"x": 30, "y": 0}, {"x": 40, "y": 0}], tracker.history(2)["points"])

    def test_new_model_does_not_expose_old_area_history(self):
        with temporary_directory() as root:
            origin = [{"id": "p1", "lat": 32, "lon": 118}]
            self._write_model(root, points=origin)
            tracker = ModelingPositionHistory(root, now=lambda: 100)
            tracker.update(32, 118, True, force_context=True)
            self.assertEqual(1, len(tracker.history(1)["points"]))
            self._write_model(root, model_id="model-2", points=[])
            tracker.update(32, 118, True, force_context=True)
            self.assertEqual([], tracker.history()["points"])
            self.assertEqual([], tracker.history(1)["points"])
            self.assertFalse(tracker.history(1)["coordinateReady"])

    def test_legacy_points_are_not_guessed_into_an_area(self):
        with temporary_directory() as root:
            self._write_model(root, points=[{"id": "p1", "lat": 32, "lon": 118}])
            tracker = ModelingPositionHistory(root, now=lambda: 100)
            tracker.update(32, 118, True, force_context=True)
            tracker.history()
            path = os.path.join(root, "position_history", "model-1.json")
            with io.open(path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            payload.update({"version": 1, "points": [{"x": 10, "y": 5}]})
            write_json(path, payload)
            restarted = ModelingPositionHistory(root, now=lambda: 101)
            restarted.update(32, 118, False, force_context=True)
            self.assertEqual([{"x": 10, "y": 5}], restarted.history()["points"])
            self.assertEqual([], restarted.history(1)["points"])

    def test_capture_mode_wins_over_last_edited_point_type(self):
        draft = {"groups": [{"id": "g2", "areaNumber": 2}],
                 "groupLinks": [{"id": "l1", "linkNumber": 1}]}
        state = {"status": "recording", "currentGroupId": "g2", "captureMode": None,
                 "currentPointType": "link"}
        self.assertEqual({"pointType": "area", "areaNumber": 2}, history_capture_metadata(state, draft))
        state.update({"captureMode": "link", "currentPointType": "area", "currentLinkId": "l1"})
        self.assertEqual({"pointType": "link", "linkNumber": 1}, history_capture_metadata(state, draft))

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

    def test_default_history_policy_is_one_second_and_1500_points(self):
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
                flush_interval=0.0,
            )
            self.assertEqual(1.0, tracker.history_interval)
            self.assertEqual(1500, tracker.max_points)

            tracker.update(lat, lon, True, force_context=True)
            clock[0] += 0.9
            tracker.update(lat, lon + longitude_offset_cm(lat, 10), True)
            self.assertEqual([{"x": 0, "y": 0}], tracker.history()["points"])

            clock[0] += 0.1
            tracker.update(lat, lon + longitude_offset_cm(lat, 20), True)
            self.assertEqual(
                [{"x": 0, "y": 0}, {"x": 20, "y": 0}],
                tracker.history()["points"],
            )

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
