# coding=utf-8

import base64
import json
import os
import select
import socket
import threading
import time


def _to_bool(value, default=False):
    if value is None:
        return default
    return str(value).strip().lower() not in ('0', 'false', 'off', '')


class NtripConfig(object):
    def __init__(self, enabled=False, host='', port=0, mountpoint='', username='',
                 password='', gga_interval_seconds=5.0, connect_timeout_seconds=5.0,
                 reconnect_interval_seconds=5.0):
        self.enabled = enabled
        self.host = host
        self.port = int(port or 0)
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.gga_interval_seconds = float(gga_interval_seconds)
        self.connect_timeout_seconds = float(connect_timeout_seconds)
        self.reconnect_interval_seconds = float(reconnect_interval_seconds)

    def is_complete(self):
        return bool(
            self.enabled and self.host and self.port > 0 and self.mountpoint and self.username and self.password
        )

    @classmethod
    def from_sources(cls, config_file='ntrip_config.json'):
        file_data = {}
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as fp:
                    file_data = json.load(fp)
            except Exception:
                file_data = {}

        def pick(name, default=None):
            env_name = 'CLEANER_NTRIP_' + name.upper()
            if os.environ.get(env_name) not in (None, ''):
                return os.environ.get(env_name)
            return file_data.get(name, default)

        return cls(
            enabled=_to_bool(pick('enabled', False), False),
            host=(pick('host', '') or '').strip(),
            port=pick('port', 0) or 0,
            mountpoint=(pick('mountpoint', '') or '').strip(),
            username=(pick('username', '') or '').strip(),
            password=(pick('password', '') or '').strip(),
            gga_interval_seconds=pick('gga_interval_seconds', 5.0) or 5.0,
            connect_timeout_seconds=pick('connect_timeout_seconds', 5.0) or 5.0,
            reconnect_interval_seconds=pick('reconnect_interval_seconds', 5.0) or 5.0,
        )


def extract_gga_quality(nmea_sentence):
    if not nmea_sentence.startswith(('$GNGGA', '$GPGGA')):
        return None
    parts = nmea_sentence.strip().split(',')
    if len(parts) < 7:
        return None
    quality = parts[6].strip()
    return quality or None


def normalize_gga_sentence(nmea_sentence):
    line = nmea_sentence.strip()
    if not line.endswith('\r\n'):
        line += '\r\n'
    return line


class NtripCorrectionRuntime(object):
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger
        self._sock = None
        self._latest_gga = None
        self._last_gga_sent_at = 0.0
        self._last_connect_attempt_at = 0.0
        self._connected_once = False

    def observe_gga(self, nmea_sentence):
        quality = extract_gga_quality(nmea_sentence)
        if quality is None or quality == '0':
            return
        self._latest_gga = normalize_gga_sentence(nmea_sentence)

    def prepare(self):
        """在读线程启动前预建立NTRIP差分链路。失败不抛异常,返回是否已连接。"""
        if not self.config.enabled:
            return False
        if not self.config.is_complete():
            self.logger.warning('NTRIP配置不完整,跳过差分预连接')
            return False
        self._ensure_connected()
        return self._sock is not None

    def step(self, serial_port):
        if not self.config.enabled:
            return
        if not self.config.is_complete():
            self.logger.warning('NTRIP配置不完整，跳过差分转发')
            return
        self._ensure_connected()
        if self._sock is None:
            return
        self._maybe_send_gga()
        self._forward_rtcm(serial_port)

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def _ensure_connected(self):
        if self._sock is not None:
            return
        now = time.time()
        if (now - self._last_connect_attempt_at) < self.config.reconnect_interval_seconds:
            return
        self._last_connect_attempt_at = now
        try:
            sock = socket.create_connection(
                (self.config.host, self.config.port),
                self.config.connect_timeout_seconds,
            )
            sock.settimeout(self.config.connect_timeout_seconds)
            sock.sendall(self._build_request_header())
            response = sock.recv(1024)
            first_line = response.splitlines()[0] if response else ''
            if isinstance(first_line, bytes):
                first_line_text = first_line.decode('utf-8', errors='ignore')
            else:
                first_line_text = first_line
            if '200' not in first_line_text:
                raise RuntimeError('NTRIP响应异常: %s' % first_line_text)
            sock.setblocking(False)
            self._sock = sock
            if not self._connected_once:
                self.logger.warning(
                    'NTRIP差分链路已连接: %s:%s/%s',
                    self.config.host,
                    self.config.port,
                    self.config.mountpoint,
                )
                self._connected_once = True
        except Exception as exc:
            self.close()
            self.logger.error('NTRIP连接失败: %s', exc)

    def _build_request_header(self):
        user_pwd = ('%s:%s' % (self.config.username, self.config.password)).encode('utf-8')
        auth = base64.b64encode(user_pwd).decode('ascii')
        header = (
            'GET /%s HTTP/1.0\r\n'
            'User-Agent: NTRIP GNSSInternetRadio/1.4.10\r\n'
            'Accept: */*\r\n'
            'Connection: close\r\n'
            'Authorization: Basic %s\r\n\r\n'
        ) % (self.config.mountpoint, auth)
        return header.encode('ascii')

    def _maybe_send_gga(self):
        if not self._latest_gga or self._sock is None:
            return
        now = time.time()
        if (now - self._last_gga_sent_at) < self.config.gga_interval_seconds:
            return
        try:
            self._sock.sendall(self._latest_gga.encode('ascii', errors='ignore'))
            self._last_gga_sent_at = now
        except Exception as exc:
            self.logger.error('发送GGA到NTRIP失败: %s', exc)
            self.close()

    def _forward_rtcm(self, serial_port):
        if self._sock is None:
            return
        try:
            readable, _, _ = select.select([self._sock], [], [], 0)
        except Exception as exc:
            self.logger.error('检查NTRIP套接字状态失败: %s', exc)
            self.close()
            return
        if not readable:
            return
        try:
            data = self._sock.recv(4096)
            if not data:
                self.logger.warning('NTRIP差分链路已断开')
                self.close()
                return
            serial_port.write(data)
        except Exception as exc:
            self.logger.error('转发RTCM到RTK串口失败: %s', exc)
            self.close()


_shared_runtime = None
_shared_runtime_lock = threading.Lock()


def get_shared_runtime(logger):
    """返回进程内共享的 NtripCorrectionRuntime 单例。"""
    global _shared_runtime
    with _shared_runtime_lock:
        if _shared_runtime is None:
            _shared_runtime = NtripCorrectionRuntime(NtripConfig.from_sources(), logger)
        return _shared_runtime

