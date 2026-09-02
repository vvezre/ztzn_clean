# coding=utf-8
"""Local account authentication for the robot LAN HTTP facade.

The cloud service exposes ``/auth/login``, ``/auth/refresh`` and
``/auth/logout``.  This module provides the same contracts without requiring
Internet access.  Passwords are stored as salted PBKDF2-SHA256 records and
tokens use the standard JWT HS256 wire format, implemented with Python's
standard library so the Jetson Python 2.7 runtime needs no new dependency.
"""

from __future__ import absolute_import

import base64
import datetime
import hashlib
import hmac
import json
import os
import threading
import time
import uuid

from flask import jsonify, request


DEFAULT_ACCESS_SECONDS = 3600
DEFAULT_REFRESH_SECONDS = 604800
DEFAULT_PASSWORD_ITERATIONS = 120000


try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


def _text(value):
    if value is None:
        return ''
    if isinstance(value, text_type):
        return value
    if isinstance(value, binary_type):
        try:
            return value.decode('utf-8')
        except Exception:
            return value.decode('utf-8', 'replace')
    try:
        return text_type(value)
    except Exception:
        return ''


def _bytes(value):
    if isinstance(value, binary_type):
        return value
    if isinstance(value, text_type):
        return value.encode('utf-8')
    return _text(value).encode('utf-8')


def _b64url_encode(value):
    encoded = base64.urlsafe_b64encode(value).rstrip(b'=')
    return encoded.decode('ascii') if not isinstance(encoded, text_type) else encoded


def _b64url_decode(value):
    raw = _bytes(value)
    raw += b'=' * ((4 - len(raw) % 4) % 4)
    return base64.urlsafe_b64decode(raw)


def _constant_time_equal(left, right):
    left = _bytes(left)
    right = _bytes(right)
    compare_digest = getattr(hmac, 'compare_digest', None)
    if compare_digest is not None:
        return compare_digest(left, right)
    if len(left) != len(right):
        return False
    difference = 0
    for left_byte, right_byte in zip(bytearray(left), bytearray(right)):
        difference |= left_byte ^ right_byte
    return difference == 0


def _pbkdf2(password, salt, iterations):
    implementation = getattr(hashlib, 'pbkdf2_hmac', None)
    if implementation is None:
        raise RuntimeError('Python runtime does not provide hashlib.pbkdf2_hmac')
    return implementation('sha256', _bytes(password), salt, int(iterations))


def create_password_record(password, iterations=DEFAULT_PASSWORD_ITERATIONS, salt=None):
    """Return a non-reversible PBKDF2 password record suitable for JSON."""
    if not _text(password):
        raise ValueError('password must not be empty')
    salt = salt or os.urandom(16)
    digest = _pbkdf2(password, salt, iterations)
    return 'pbkdf2_sha256${}${}${}'.format(
        int(iterations),
        _b64url_encode(salt),
        _b64url_encode(digest),
    )


def verify_password(password, record):
    try:
        algorithm, iterations, salt_text, expected_text = _text(record).split('$', 3)
        if algorithm != 'pbkdf2_sha256':
            return False
        actual = _pbkdf2(password, _b64url_decode(salt_text), int(iterations))
        return _constant_time_equal(actual, _b64url_decode(expected_text))
    except Exception:
        return False


def default_auth_config_path():
    configured = os.environ.get('CLEANBOT_LAN_AUTH_FILE')
    if configured:
        return os.path.abspath(configured)
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lan_auth.json')


def load_auth_config(path=None):
    path = os.path.abspath(path or default_auth_config_path())
    if not os.path.isfile(path):
        return None, path, '局域网账号尚未配置'
    try:
        with open(path, 'rb') as handle:
            raw = handle.read()
        config = json.loads(raw.decode('utf-8-sig'))
    except Exception as exc:
        return None, path, '局域网账号配置读取失败: {}'.format(exc)
    if not isinstance(config, dict):
        return None, path, '局域网账号配置格式错误'
    return config, path, None


def _result_payload(code, message, data=None):
    result = {
        'code': int(code),
        'message': message,
        'timestamp': datetime.datetime.now().isoformat(),
    }
    if data is not None:
        result['data'] = data
    return result


class LocalAuthManager(object):
    """Validate local users and issue cloud-shaped access/refresh tokens."""

    def __init__(self, config=None, config_path=None, now_provider=None):
        self.config_path = os.path.abspath(config_path) if config_path else default_auth_config_path()
        self.now_provider = now_provider or time.time
        self._lock = threading.RLock()
        self._revoked_access = {}
        self._active_refresh_by_user = {}
        self.available = False
        self.unavailable_reason = '局域网账号尚未配置'
        self.secret = b''
        self.access_seconds = DEFAULT_ACCESS_SECONDS
        self.refresh_seconds = DEFAULT_REFRESH_SECONDS
        self.users = []
        if config is None:
            config, self.config_path, error = load_auth_config(self.config_path)
            if error:
                self.unavailable_reason = error
                return
        self._apply_config(config)

    def _apply_config(self, config):
        if not isinstance(config, dict):
            self.unavailable_reason = '局域网账号配置格式错误'
            return
        secret = _text(config.get('tokenSecret')).strip()
        users = config.get('users')
        if len(secret) < 32:
            self.unavailable_reason = '局域网Token密钥长度不足'
            return
        if not isinstance(users, list) or not users:
            self.unavailable_reason = '局域网账号列表为空'
            return
        normalized_users = []
        for item in users:
            if not isinstance(item, dict):
                continue
            username = _text(item.get('username')).strip()
            password_hash = _text(item.get('passwordHash')).strip()
            if not username or not password_hash:
                continue
            normalized_users.append({
                'userId': int(item.get('userId') or len(normalized_users) + 1),
                'username': username,
                'passwordHash': password_hash,
                'realName': _text(item.get('realName') or username),
                'roleId': int(item.get('roleId') or 1),
                'roleName': _text(item.get('roleName') or 'admin'),
                'permissions': item.get('permissions') if isinstance(item.get('permissions'), list) else [],
                'status': _text(item.get('status') or 'enable').lower(),
            })
        if not normalized_users:
            self.unavailable_reason = '局域网账号配置中没有有效账号'
            return
        self.secret = _bytes(secret)
        self.access_seconds = max(60, int(config.get('accessTokenSeconds') or DEFAULT_ACCESS_SECONDS))
        self.refresh_seconds = max(
            self.access_seconds,
            int(config.get('refreshTokenSeconds') or DEFAULT_REFRESH_SECONDS),
        )
        self.users = normalized_users
        self.available = True
        self.unavailable_reason = ''

    def _find_user(self, username):
        username = _text(username).strip()
        for user in self.users:
            if user.get('username') == username:
                return user
        return None

    def _claims(self, user, token_type, lifetime_seconds):
        issued_at = int(self.now_provider())
        return {
            'sub': user.get('username'),
            'userId': user.get('userId'),
            'roleId': user.get('roleId'),
            'roleName': user.get('roleName'),
            'permissions': user.get('permissions') or [],
            'type': token_type,
            'iat': issued_at,
            'exp': issued_at + int(lifetime_seconds),
            'jti': uuid.uuid4().hex,
        }

    def _encode_token(self, claims):
        header = {'alg': 'HS256', 'typ': 'JWT'}
        header_text = json.dumps(header, separators=(',', ':'), sort_keys=True)
        claims_text = json.dumps(claims, separators=(',', ':'), sort_keys=True)
        signing_input = '{}.{}'.format(
            _b64url_encode(_bytes(header_text)),
            _b64url_encode(_bytes(claims_text)),
        )
        signature = hmac.new(self.secret, _bytes(signing_input), hashlib.sha256).digest()
        return '{}.{}'.format(signing_input, _b64url_encode(signature))

    def _decode_token(self, token, expected_type):
        if not self.available:
            return None, self.unavailable_reason
        try:
            header_part, payload_part, signature_part = _text(token).split('.', 2)
            signing_input = '{}.{}'.format(header_part, payload_part)
            expected_signature = hmac.new(
                self.secret,
                _bytes(signing_input),
                hashlib.sha256,
            ).digest()
            if not _constant_time_equal(expected_signature, _b64url_decode(signature_part)):
                return None, 'Token无效或已过期'
            header = json.loads(_b64url_decode(header_part).decode('utf-8'))
            claims = json.loads(_b64url_decode(payload_part).decode('utf-8'))
            if header.get('alg') != 'HS256' or claims.get('type') != expected_type:
                return None, 'Token无效或已过期'
            if int(claims.get('exp') or 0) <= int(self.now_provider()):
                return None, 'Token无效或已过期'
            user = self._find_user(claims.get('sub'))
            if user is None or user.get('status') == 'disable':
                return None, '用户不存在或已禁用'
            if expected_type == 'access':
                with self._lock:
                    revoked_until = self._revoked_access.get(_text(claims.get('jti')))
                if revoked_until and revoked_until > int(self.now_provider()):
                    return None, 'Token无效或已过期'
            return claims, None
        except Exception:
            return None, 'Token无效或已过期'

    def login(self, username, password):
        if not self.available:
            return None, 503, self.unavailable_reason
        user = self._find_user(username)
        if user is None or not verify_password(password, user.get('passwordHash')):
            return None, 401, '用户名或密码错误'
        if user.get('status') == 'disable':
            return None, 403, '账户已被禁用'
        return self._login_response(user), 200, '成功'

    def _login_response(self, user, refresh_token=None):
        access_claims = self._claims(user, 'access', self.access_seconds)
        if refresh_token is None:
            refresh_claims = self._claims(user, 'refresh', self.refresh_seconds)
            refresh_token = self._encode_token(refresh_claims)
            with self._lock:
                self._active_refresh_by_user[user.get('userId')] = refresh_claims.get('jti')
        return {
            'accessToken': self._encode_token(access_claims),
            'refreshToken': refresh_token,
            'expiresIn': self.access_seconds,
            'user': {
                'userId': user.get('userId'),
                'username': user.get('username'),
                'realName': user.get('realName'),
                'roleId': user.get('roleId'),
                'roleName': user.get('roleName'),
            },
        }

    def verify_authorization(self, authorization, token_type='access'):
        authorization = _text(authorization).strip()
        if not authorization.startswith('Bearer '):
            return None, 401, '未登录或登录已过期'
        claims, error = self._decode_token(authorization[7:].strip(), token_type)
        if error:
            return None, 401 if self.available else 503, error
        return claims, 200, '成功'

    def refresh(self, authorization):
        claims, code, message = self.verify_authorization(authorization, 'refresh')
        if claims is None:
            return None, code, message
        user = self._find_user(claims.get('sub'))
        if user is None or user.get('status') == 'disable':
            return None, 403, '用户不存在或已禁用'
        with self._lock:
            active_jti = self._active_refresh_by_user.get(user.get('userId'))
        if active_jti != claims.get('jti'):
            return None, 401, 'Refresh Token已失效'
        refresh_token = _text(authorization).strip()[7:].strip()
        return self._login_response(user, refresh_token=refresh_token), 200, '成功'

    def logout(self, authorization):
        claims, code, message = self.verify_authorization(authorization, 'access')
        if claims is None:
            return False, code, message
        with self._lock:
            self._revoked_access[_text(claims.get('jti'))] = int(claims.get('exp') or 0)
            self._active_refresh_by_user.pop(claims.get('userId'), None)
            now = int(self.now_provider())
            self._revoked_access = dict(
                (key, expiry) for key, expiry in self._revoked_access.items()
                if expiry > now
            )
        return True, 200, '登出成功'


def register_lan_auth_routes(app, auth_manager):
    """Register cloud-compatible local authentication endpoints."""

    @app.route('/auth/login', methods=['POST'])
    def lan_auth_login():
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            return jsonify(_result_payload(400, '请求体必须是JSON对象')), 400
        data, code, message = auth_manager.login(
            payload.get('username'),
            payload.get('password'),
        )
        # The cloud AuthController returns Result.error in a normal controller
        # response, so credential errors use HTTP 200 and carry code=401/403.
        http_status = 503 if code == 503 else 200
        return jsonify(_result_payload(code, message, data)), http_status

    @app.route('/auth/refresh', methods=['POST'])
    def lan_auth_refresh():
        data, code, message = auth_manager.refresh(request.headers.get('Authorization'))
        http_status = code if code == 503 else 200
        return jsonify(_result_payload(code, message, data)), http_status

    @app.route('/auth/logout', methods=['POST'])
    def lan_auth_logout():
        succeeded, code, message = auth_manager.logout(request.headers.get('Authorization'))
        if not succeeded:
            return jsonify(_result_payload(code, message)), code
        return jsonify(_result_payload(200, '成功', '登出成功'))

    return auth_manager


def auth_error_response(code, message):
    return jsonify(_result_payload(code, message)), int(code)
