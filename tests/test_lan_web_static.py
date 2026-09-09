# coding=utf-8
import io
import os
import shutil
import tempfile
import unittest

from flask import Flask

from lan_cloud_compat import register_lan_cloud_compat_routes
from lan_local_auth import LocalAuthManager, create_password_record


class LanWebStaticTests(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='cleanbot-web-')
        for directory in ('js', 'css', 'chunk', 'static'):
            os.makedirs(os.path.join(self.root, directory))
        self.files = {
            'index.html': u'<!doctype html><title>中拓智能</title>',
            'js/app.js': u'window.cleanbot = true;',
            'css/app.css': u'body { color: #123; }',
            'chunk/825.js': u'window.chunk825 = true;',
            'static/logo.txt': u'cleanbot-logo',
        }
        for relative, content in self.files.items():
            with io.open(os.path.join(self.root, *relative.split('/')), 'w', encoding='utf-8') as handle:
                handle.write(content)

        app = Flask(__name__)
        auth = LocalAuthManager(config={
            'tokenSecret': 'test-secret-must-have-at-least-32-characters',
            'users': [{
                'userId': 1,
                'username': 'admin',
                'passwordHash': create_password_record('test-password'),
                'realName': '本地管理员',
                'roleId': 1,
                'roleName': 'admin',
                'permissions': [],
                'status': 'enable',
            }],
        })
        register_lan_cloud_compat_routes(
            app,
            lambda: None,
            auth_manager=auth,
            device_identity_provider=lambda: {
                'productId': '250006', 'productType': '-T01',
            },
            web_root=self.root,
        )
        self.client = app.test_client()

    def tearDown(self):
        shutil.rmtree(self.root)

    def test_index_aliases_and_all_h5_asset_directories_are_served(self):
        for path in ('/', '/index', '/index.html'):
            response = self.client.get(path)
            self.assertEqual(200, response.status_code)
            self.assertIn(u'中拓智能', response.get_data(as_text=True))
        for relative, content in self.files.items():
            if relative == 'index.html':
                continue
            response = self.client.get('/' + relative)
            self.assertEqual(200, response.status_code, relative)
            self.assertEqual(content, response.get_data(as_text=True))

    def test_web_is_public_but_existing_lan_api_is_still_authenticated(self):
        self.assertEqual(200, self.client.get('/').status_code)
        self.assertEqual(
            401,
            self.client.get('/api/t-railcar/saved-routes/250006').status_code,
        )
        login = self.client.post('/auth/login', json={
            'username': 'admin', 'password': 'test-password',
        })
        self.assertEqual(200, login.status_code)

    def test_unknown_files_and_parent_directory_escape_are_not_exposed(self):
        self.assertEqual(404, self.client.get('/js/missing.js').status_code)
        self.assertEqual(404, self.client.get('/js/../../index.html').status_code)


if __name__ == '__main__':
    unittest.main()
