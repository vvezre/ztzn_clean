# coding=utf-8
import threading
import time

import redis


class LocalRedis(object):
    """Small in-memory Redis substitute for Windows/local UI development."""

    def __init__(self):
        self._data = {}
        self._lock = threading.RLock()

    def _to_string(self, value):
        if value is None:
            return None
        if isinstance(value, bytes):
            return value.decode('utf-8')
        return str(value)

    def set(self, key, value):
        with self._lock:
            self._data[key] = self._to_string(value)
        return True

    def get(self, key):
        with self._lock:
            value = self._data.get(key)
            return value if isinstance(value, str) else None

    def delete(self, *keys):
        removed = 0
        with self._lock:
            for key in keys:
                if key in self._data:
                    removed += 1
                    del self._data[key]
        return removed

    def incr(self, key, amount=1):
        with self._lock:
            value = int(self.get(key) or 0) + int(amount)
            self._data[key] = str(value)
            return value

    def hset(self, name, key=None, value=None, mapping=None):
        with self._lock:
            bucket = self._data.setdefault(name, {})
            if not isinstance(bucket, dict):
                bucket = {}
                self._data[name] = bucket
            changed = 0
            items = mapping or {}
            if key is not None:
                items = dict(items)
                items[key] = value
            for item_key, item_value in items.items():
                if item_key not in bucket:
                    changed += 1
                bucket[self._to_string(item_key)] = self._to_string(item_value)
            return changed

    def hget(self, name, key):
        with self._lock:
            bucket = self._data.get(name, {})
            if not isinstance(bucket, dict):
                return None
            return bucket.get(self._to_string(key))

    def hgetall(self, name):
        with self._lock:
            bucket = self._data.get(name, {})
            return dict(bucket) if isinstance(bucket, dict) else {}

    def lpush(self, name, *values):
        with self._lock:
            bucket = self._data.setdefault(name, [])
            if not isinstance(bucket, list):
                bucket = []
                self._data[name] = bucket
            for value in values:
                bucket.insert(0, self._to_string(value))
            return len(bucket)

    def rpush(self, name, *values):
        with self._lock:
            bucket = self._data.setdefault(name, [])
            if not isinstance(bucket, list):
                bucket = []
                self._data[name] = bucket
            bucket.extend(self._to_string(value) for value in values)
            return len(bucket)

    def lrange(self, name, start, end):
        with self._lock:
            bucket = self._data.get(name, [])
            if not isinstance(bucket, list):
                return []
            stop = None if int(end) == -1 else int(end) + 1
            return list(bucket[int(start):stop])

    def lindex(self, name, index):
        with self._lock:
            bucket = self._data.get(name, [])
            if not isinstance(bucket, list):
                return None
            try:
                return bucket[int(index)]
            except IndexError:
                return None

    def lpop(self, name):
        with self._lock:
            bucket = self._data.get(name, [])
            if not isinstance(bucket, list) or not bucket:
                return None
            return bucket.pop(0)

    def llen(self, name):
        with self._lock:
            bucket = self._data.get(name, [])
            return len(bucket) if isinstance(bucket, list) else 0

    def sadd(self, name, *values):
        with self._lock:
            bucket = self._data.setdefault(name, set())
            if not isinstance(bucket, set):
                bucket = set()
                self._data[name] = bucket
            before = len(bucket)
            bucket.update(self._to_string(value) for value in values)
            return len(bucket) - before

    def smembers(self, name):
        with self._lock:
            bucket = self._data.get(name, set())
            return set(bucket) if isinstance(bucket, set) else set()

    def brpop(self, name, timeout=0):
        if timeout:
            time.sleep(float(timeout))
        return None


_LOCAL_CLIENT = LocalRedis()


def create_redis_client(host='localhost', port=6379, db=0, decode_responses=True, local_mode=False):
    if local_mode:
        return _LOCAL_CLIENT
    return redis.Redis(host=host, port=port, db=db, decode_responses=decode_responses)
