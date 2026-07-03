import shutil
import tempfile
import unittest

from flask import Flask


class ModelingRoutesTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        app = Flask(__name__)
        from modeling_routes import register_modeling_routes
        register_modeling_routes(app, storage_dir=self.tmpdir)
        self.client = app.test_client()

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_create_list_load_save_and_delete_model(self):
        created = self.client.post("/modeling/models", json={"name": "field-a"})
        self.assertEqual(created.status_code, 200)
        model = created.get_json()["data"]
        self.assertEqual(model["name"], "field-a")

        listed = self.client.get("/modeling/models")
        self.assertEqual(listed.status_code, 200)
        self.assertEqual(listed.get_json()["data"][0]["id"], model["id"])

        loaded = self.client.get("/modeling/models/" + model["id"])
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.get_json()["data"]["id"], model["id"])

        saved = self.client.post("/modeling/models/{}/save".format(model["id"]))
        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["status"], "saved")

        deleted = self.client.delete("/modeling/models/" + model["id"])
        self.assertEqual(deleted.status_code, 200)
        self.assertTrue(deleted.get_json()["success"])

    def test_save_and_read_draft(self):
        created = self.client.post("/modeling/models", json={"name": "draft-a"}).get_json()["data"]
        draft = dict(created)
        draft["groups"] = [{"id": "g1", "name": "group-1", "points": []}]

        saved = self.client.post("/modeling/draft", json={"modelId": created["id"], "draft": draft})
        self.assertEqual(saved.status_code, 200)
        self.assertEqual(saved.get_json()["data"]["groups"][0]["id"], "g1")

        loaded = self.client.get("/modeling/draft/" + created["id"])
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.get_json()["data"]["groups"][0]["name"], "group-1")

    def test_missing_model_returns_404_response(self):
        response = self.client.get("/modeling/models/missing")

        self.assertEqual(response.status_code, 404)
        self.assertFalse(response.get_json()["success"])
        self.assertEqual(response.get_json()["code"], "MODEL_NOT_FOUND")

    def test_invalid_payload_returns_400_response(self):
        response = self.client.post("/modeling/draft", json={"modelId": "", "draft": []})

        self.assertEqual(response.status_code, 400)
        self.assertFalse(response.get_json()["success"])


if __name__ == "__main__":
    unittest.main()
