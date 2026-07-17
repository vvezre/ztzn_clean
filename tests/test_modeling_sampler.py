import unittest


class ModelingSamplerTest(unittest.TestCase):
    def test_sampler_averages_fixed_rtk_samples(self):
        from modeling_sampler import sample_current_point

        samples = iter([
            {
                "lat": 32.0,
                "lon": 118.0,
                "heading": 90.0,
                "rtkQuality": "4",
                "rtkGgaAgeSec": 0.2,
                "controlState": "READY",
                "moving": False,
            },
            {
                "lat": 32.0000001,
                "lon": 118.0000001,
                "heading": 92.0,
                "rtkQuality": "4",
                "rtkGgaAgeSec": 0.2,
                "controlState": "READY",
                "moving": False,
            },
        ])

        point = sample_current_point(lambda: next(samples), sample_count=2, now=lambda: 1000)

        self.assertEqual(point["source"], "rtk_mean")
        self.assertEqual(point["sample"]["count"], 2)
        self.assertAlmostEqual(point["lat"], 32.00000005, places=8)
        self.assertAlmostEqual(point["lon"], 118.00000005, places=8)
        self.assertAlmostEqual(point["heading"], 91.0, places=6)

    def test_sampler_rejects_unfixed_rtk(self):
        from modeling_sampler import ModelingSampleError, sample_current_point

        with self.assertRaises(ModelingSampleError) as ctx:
            sample_current_point(lambda: {
                "lat": 32.0,
                "lon": 118.0,
                "rtkQuality": "2",
                "rtkGgaAgeSec": 0.1,
                "controlState": "READY",
            }, sample_count=1)

        self.assertEqual(ctx.exception.code, "RTK_NOT_FIXED")

    def test_sampler_rejects_moving_vehicle(self):
        from modeling_sampler import ModelingSampleError, sample_current_point

        with self.assertRaises(ModelingSampleError) as ctx:
            sample_current_point(lambda: {
                "lat": 32.0,
                "lon": 118.0,
                "rtkQuality": "4",
                "rtkGgaAgeSec": 0.1,
                "controlState": "RUNNING",
            }, sample_count=1)

        self.assertEqual(ctx.exception.code, "VEHICLE_NOT_STATIC")

    def test_sample_readiness_reports_missing_location_without_sampling(self):
        from modeling_sampler import inspect_sample_readiness

        status = inspect_sample_readiness(lambda: {
            "lat": None,
            "lon": None,
            "rtkQuality": "4",
            "rtkGgaAgeSec": 0.1,
            "controlState": "READY",
        })

        self.assertFalse(status["ready"])
        self.assertEqual(status["code"], "RTK_LOCATION_MISSING")
        self.assertIn("无RTK坐标", status["message"])

    def test_sample_readiness_reports_ready_when_static_and_fixed(self):
        from modeling_sampler import inspect_sample_readiness

        status = inspect_sample_readiness(lambda: {
            "lat": 32.0,
            "lon": 118.0,
            "rtkQuality": "4",
            "rtkGgaAgeSec": 0.1,
            "controlState": "READY",
            "moving": False,
        })

        self.assertTrue(status["ready"])
        self.assertEqual(status["code"], "READY")
        self.assertIn("可以记录", status["message"])


if __name__ == "__main__":
    unittest.main()
