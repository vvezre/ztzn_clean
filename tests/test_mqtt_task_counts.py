import unittest

from mqtt_integration import MQTTIntegration


class FakeRedis(object):
    def __init__(self):
        self.values = {}
        self.hashes = {}
        self.lists = {}

    def set(self, key, value):
        self.values[key] = str(value)

    def get(self, key):
        return self.values.get(key)

    def hgetall(self, key):
        return dict(self.hashes.get(key, {}))

    def rpush(self, key, value):
        self.lists.setdefault(key, []).append(value)

    def llen(self, key):
        return len(self.lists.get(key, []))

    def hset(self, key, field, value):
        self.hashes.setdefault(key, {})[field] = str(value)


class MQTTTaskCountsTest(unittest.TestCase):
    def test_status_keeps_clean_task_count_separate_from_waypoint_count(self):
        redis_client = FakeRedis()
        redis_client.set("currentAction", "multi_go_to_point")
        redis_client.set("mission", "working")
        redis_client.set("waypointTotal", 8)
        redis_client.set("waypointIndex", 2)
        redis_client.set("curTaskIndex", 5)
        for index in range(8):
            redis_client.rpush("waypoints", index)

        integration = MQTTIntegration.__new__(MQTTIntegration)
        integration.redis_client = redis_client
        integration._load_task_config = lambda: {"taskList": [{} for _ in range(12)]}

        status = integration._get_vehicle_status_from_redis()

        self.assertEqual(status["task_count"], 12)
        self.assertEqual(status["cur_task_index"], 5)
        self.assertEqual(status["waypoint_count"], 8)
        self.assertEqual(status["waypoint_index"], 2)

    def test_status_prefers_runtime_state_over_legacy_mission_fields(self):
        redis_client = FakeRedis()
        redis_client.set("mission", "complete")
        redis_client.set("parking", "1")
        redis_client.set("currentAction", "idle")
        redis_client.set("controlState", "STOPPED")
        redis_client.set("runtimeState", """
        {
          "state": "COMPLETE",
          "controlState": "COMPLETE",
          "action": "auto_drive",
          "health": "OK",
          "fault": "",
          "mission": "complete",
          "parking": true,
          "detail": {"rtkFixState": "FIXED", "stateSource": "runtimeStateDetail"}
        }
        """)

        integration = MQTTIntegration.__new__(MQTTIntegration)
        integration.redis_client = redis_client
        integration._load_task_config = lambda: {"taskList": []}

        status = integration._get_vehicle_status_from_redis()

        self.assertEqual(status["status"], "idle")
        self.assertEqual(status["mission_state"], "COMPLETE")
        self.assertEqual(status["control_state"], "COMPLETE")
        self.assertEqual(status["action"], "auto_drive")
        self.assertEqual(status["health_state"], "OK")
        self.assertEqual(status["fault_state"], "")
        self.assertEqual(status["detail"]["stateSource"], "runtimeStateDetail")

    def test_health_state_prefers_runtime_error_over_fault_fallback(self):
        redis_client = FakeRedis()
        integration = MQTTIntegration.__new__(MQTTIntegration)
        integration.redis_client = redis_client

        health_state = integration._build_health_state({
            "health": "ERROR",
            "fault": "MOTOR_FAULT",
        })

        self.assertEqual(health_state, "ERROR")


if __name__ == "__main__":
    unittest.main()
