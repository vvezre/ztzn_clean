# coding=utf-8
"""Create a local LAN account configuration without storing plaintext passwords."""

from __future__ import absolute_import, print_function

import argparse
import base64
import getpass
import json
import os
import tempfile

from lan_local_auth import create_password_record, default_auth_config_path


def build_config(username, password, real_name=None):
    return {
        'tokenSecret': base64.urlsafe_b64encode(os.urandom(48)).decode('ascii'),
        'accessTokenSeconds': 3600,
        'refreshTokenSeconds': 604800,
        'users': [{
            'userId': 1,
            'username': username,
            'passwordHash': create_password_record(password),
            'realName': real_name or username,
            'roleId': 1,
            'roleName': 'admin',
            'permissions': [],
            'status': 'enable',
        }],
    }


def write_config(path, config):
    path = os.path.abspath(path)
    target_dir = os.path.dirname(path)
    if not os.path.isdir(target_dir):
        os.makedirs(target_dir)
    descriptor, temporary_path = tempfile.mkstemp(prefix='.lan-auth-', dir=target_dir)
    try:
        with os.fdopen(descriptor, 'wb') as handle:
            body = json.dumps(config, ensure_ascii=False, indent=2, sort_keys=True)
            if not isinstance(body, bytes):
                body = body.encode('utf-8')
            handle.write(body)
            handle.flush()
            os.fsync(handle.fileno())
        if os.path.isfile(path):
            os.remove(path)
        os.rename(temporary_path, path)
        try:
            os.chmod(path, 0o600)
        except Exception:
            pass
    finally:
        if os.path.exists(temporary_path):
            os.remove(temporary_path)


def main():
    parser = argparse.ArgumentParser(description='Configure robot LAN login account')
    parser.add_argument('--username', default='admin')
    parser.add_argument('--real-name', default='本地管理员')
    parser.add_argument('--output', default=default_auth_config_path())
    args = parser.parse_args()
    password = getpass.getpass('Local LAN password: ')
    confirmation = getpass.getpass('Confirm password: ')
    if not password or password != confirmation:
        raise SystemExit('password is empty or confirmation does not match')
    write_config(args.output, build_config(args.username, password, args.real_name))
    print('LAN account configuration written to {}'.format(os.path.abspath(args.output)))


if __name__ == '__main__':
    main()
