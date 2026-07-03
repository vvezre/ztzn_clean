import os
import shutil
import tempfile
import unittest


class ModelingStoreTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_create_model_writes_formal_model_and_empty_draft(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)

        model = store.create_model("field-model-a")

        self.assertEqual(model["name"], "field-model-a")
        self.assertEqual(model["status"], "draft")
        self.assertEqual(model["version"], 1)
        self.assertEqual(model["groups"], [])
        self.assertTrue(os.path.exists(os.path.join(self.tmpdir, "models", model["id"] + ".json")))
        self.assertTrue(os.path.exists(os.path.join(self.tmpdir, "drafts", model["id"] + ".json")))

    def test_save_draft_updates_draft_without_marking_formal_model_saved(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("draft-model")
        draft = dict(model)
        draft["groups"] = [{"id": "g1", "name": "group-1", "points": []}]

        saved = store.save_draft(model["id"], draft, now=1005)
        formal = store.get_model(model["id"])

        self.assertEqual(saved["groups"][0]["id"], "g1")
        self.assertEqual(saved["status"], "draft")
        self.assertEqual(saved["updatedAt"], 1005)
        self.assertEqual(formal["groups"], [])

    def test_save_model_promotes_draft_to_formal_model(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("saved-model")
        draft = dict(model)
        draft["groups"] = [{"id": "g1", "name": "group-1", "points": []}]
        store.save_draft(model["id"], draft, now=1005)

        saved = store.save_model(model["id"], now=1010)

        self.assertEqual(saved["status"], "saved")
        self.assertEqual(saved["savedAt"], 1010)
        self.assertEqual(store.get_model(model["id"])["groups"][0]["id"], "g1")

    def test_list_models_returns_saved_metadata_without_large_body(self):
        from modeling_store import ModelingStore

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("list-model")
        store.save_model(model["id"], now=1010)

        items = store.list_models()

        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["id"], model["id"])
        self.assertEqual(items[0]["name"], "list-model")
        self.assertEqual(items[0]["status"], "saved")
        self.assertNotIn("groups", items[0])

    def test_delete_model_removes_model_and_draft_files(self):
        from modeling_store import ModelingStore, ModelNotFoundError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)
        model = store.create_model("delete-model")

        removed = store.delete_model(model["id"])

        self.assertTrue(removed)
        with self.assertRaises(ModelNotFoundError):
            store.get_model(model["id"])
        self.assertFalse(os.path.exists(os.path.join(self.tmpdir, "drafts", model["id"] + ".json")))

    def test_rejects_unsafe_model_id(self):
        from modeling_store import ModelingStore, InvalidModelIdError

        store = ModelingStore(self.tmpdir, now=lambda: 1000)

        with self.assertRaises(InvalidModelIdError):
            store.get_model("../bad")


if __name__ == "__main__":
    unittest.main()
