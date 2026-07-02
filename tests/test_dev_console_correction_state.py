import json
import os
import tempfile
import unittest


class FakeRedis(object):
    def __init__(self):
        self.values = {}
        self.hashes = {}
        self.lists = {}

    def set(self, key, value):
        self.values[key] = str(value)

    def get(self, key):
        return self.values.get(key)

    def hset(self, name, key, value):
        self.hashes.setdefault(name, {})[key] = str(value)

    def hgetall(self, name):
        return dict(self.hashes.get(name, {}))

    def rpush(self, name, value):
        self.lists.setdefault(name, []).append(value)

    def lindex(self, name, index):
        items = self.lists.get(name, [])
        try:
            return items[int(index)]
        except IndexError:
            return None

    def lrange(self, name, start, end):
        items = self.lists.get(name, [])
        stop = None if int(end) == -1 else int(end) + 1
        return list(items[int(start):stop])

    def keys(self, pattern="*"):
        keys = list(self.values.keys()) + list(self.hashes.keys()) + list(self.lists.keys())
        if pattern == "*":
            return keys
        return [key for key in keys if key == pattern]


class DevConsoleCorrectionStateTest(unittest.TestCase):
    def test_does_not_compute_correction_without_runtime_debug(self):
        from dev_console.correction_state import build_correction_state

        redis_client = FakeRedis()
        redis_client.set("mission", "working")
        redis_client.set("parking", "0")
        redis_client.set("runtimeDetail", json.dumps({"rtkFixAvailable": True}))
        redis_client.hset("currentLocation", "lat", 0.000001)
        redis_client.hset("currentLocation", "lon", 0.00005)
        redis_client.hset("currentLocation", "heading", 88.0)
        redis_client.hset("currentLocation", "headingAt", 999.5)
        redis_client.rpush("taskList", json.dumps({
            "id": 7,
            "startLat": 0.0,
            "startLon": 0.0,
            "endLat": 0.0,
            "endLon": 0.0001,
            "heading": 90.0,
        }))

        trace = []
        state = build_correction_state(redis_client, trace=trace, now=1000.0)

        self.assertEqual(state["segment"]["id"], 7)
        self.assertAlmostEqual(state["vehicle"]["heading"], 88.0)
        self.assertIsNone(state["correction"]["headingError"])
        self.assertIsNone(state["correction"]["cte"])
        self.assertIsNone(state["correction"]["zSpeed"])
        self.assertIsNone(state["correction"]["distanceToTarget"])
        self.assertIsNone(state["correction"]["signedRemaining"])
        self.assertEqual(state["correction"]["source"], "unreported")
        self.assertIsNone(state["parameters"]["headingGain"])
        self.assertIsNone(state["parameters"]["cteGain"])
        self.assertEqual(state["parameters"]["source"], "unreported")
        self.assertTrue(state["status"]["rtkFixed"])
        self.assertEqual(len(state["trace"]), 1)
        self.assertEqual(state["trace"][0]["timestamp"], 1000.0)

    def test_returns_empty_correction_when_task_is_missing(self):
        from dev_console.correction_state import build_correction_state

        redis_client = FakeRedis()
        redis_client.hset("currentLocation", "lat", 0.000001)
        redis_client.hset("currentLocation", "lon", 0.00005)
        redis_client.hset("currentLocation", "heading", 88.0)

        state = build_correction_state(redis_client, now=1000.0)

        self.assertIsNone(state["segment"])
        self.assertIsNone(state["correction"]["cte"])
        self.assertEqual(state["status"]["reason"], "no_active_segment")

    def test_ignores_current_location_without_runtime_timestamp(self):
        from dev_console.correction_state import build_correction_state

        redis_client = FakeRedis()
        redis_client.hset("currentLocation", "lat", 0.000001)
        redis_client.hset("currentLocation", "lon", 0.00005)
        redis_client.hset("currentLocation", "heading", 88.0)

        state = build_correction_state(redis_client, now=1000.0)

        self.assertIsNone(state["vehicle"]["lat"])
        self.assertIsNone(state["vehicle"]["lon"])
        self.assertIsNone(state["vehicle"]["heading"])
        self.assertEqual(state["trace"], [])

    def test_does_not_fall_back_to_config_task_when_redis_task_list_is_empty(self):
        from dev_console.correction_state import build_correction_state

        redis_client = FakeRedis()
        redis_client.set("curTaskIndex", "1")
        redis_client.hset("currentLocation", "lat", 0.0)
        redis_client.hset("currentLocation", "lon", 0.00015)
        redis_client.hset("currentLocation", "heading", 90.0)
        config = {
            "taskList": [
                {"id": 1, "startLat": 0.0, "startLon": 0.0, "endLat": 0.0, "endLon": 0.0001, "heading": 90.0},
                {"id": 2, "startLat": 0.0, "startLon": 0.0001, "endLat": 0.0, "endLon": 0.0002, "heading": 90.0},
            ]
        }
        fd, path = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        try:
            with open(path, "w") as fp:
                json.dump(config, fp)

            state = build_correction_state(redis_client, config_path=path, now=1000.0)
        finally:
            os.remove(path)

        self.assertIsNone(state["segment"])
        self.assertEqual(state["status"]["segmentSource"], None)
        self.assertEqual(state["status"]["reason"], "no_active_segment")
        self.assertIsNone(state["correction"]["zSpeed"])

    def test_trace_is_bounded(self):
        from dev_console.correction_state import build_correction_state

        redis_client = FakeRedis()
        redis_client.hset("currentLocation", "lat", 0.000001)
        redis_client.hset("currentLocation", "lon", 0.00005)
        redis_client.hset("currentLocation", "heading", 88.0)
        redis_client.hset("currentLocation", "headingAt", 999.5)
        redis_client.rpush("taskList", json.dumps({
            "id": 1,
            "startLat": 0.0,
            "startLon": 0.0,
            "endLat": 0.0,
            "endLon": 0.0001,
            "heading": 90.0,
        }))
        trace = [{"lat": 0.0, "lon": 0.0, "heading": 90.0, "timestamp": index} for index in range(5)]

        state = build_correction_state(redis_client, trace=trace, trace_limit=3, now=1000.0)

        self.assertEqual(len(state["trace"]), 3)
        self.assertEqual(state["trace"][-1]["timestamp"], 1000.0)

    def test_uses_runtime_correction_debug_when_reported(self):
        from dev_console.correction_state import build_correction_state

        redis_client = FakeRedis()
        redis_client.set("mission", "working")
        redis_client.set("parking", "0")
        redis_client.set("globalGo", "1")
        redis_client.hset("currentLocation", "lat", 0.000001)
        redis_client.hset("currentLocation", "lon", 0.00005)
        redis_client.hset("currentLocation", "heading", 88.0)
        redis_client.hset("correctionDebug", "headingError", "9.5")
        redis_client.hset("correctionDebug", "cte", "0.25")
        redis_client.hset("correctionDebug", "zSpeed", "-345")
        redis_client.hset("correctionDebug", "distanceToTarget", "4.2")
        redis_client.hset("correctionDebug", "signedRemaining", "3.8")
        redis_client.hset("correctionDebug", "headingGain", "10")
        redis_client.hset("correctionDebug", "cteGain", "1000")
        redis_client.rpush("taskList", json.dumps({
            "id": 7,
            "startLat": 0.0,
            "startLon": 0.0,
            "endLat": 0.0,
            "endLon": 0.0001,
            "heading": 90.0,
        }))

        state = build_correction_state(redis_client, now=1000.0)

        self.assertEqual(state["correction"]["source"], "runtime")
        self.assertEqual(state["correction"]["zSpeed"], -345)
        self.assertEqual(state["correction"]["headingError"], 9.5)
        self.assertEqual(state["parameters"]["headingGain"], 10.0)
        self.assertEqual(state["parameters"]["cteGain"], 1000.0)
        self.assertEqual(state["status"]["globalGoSource"], "redis")


if __name__ == "__main__":
    unittest.main()
