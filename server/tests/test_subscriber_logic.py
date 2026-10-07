import unittest
from unittest.mock import patch
import subscriberJsonOsmAnd

class TestSubscriberLogic(unittest.TestCase):
    def test_derived_speed_converted_to_knots(self):
        subscriberJsonOsmAnd._last_valid_point = {
            "lat": 0.0, "lon": 0.0, "ts": "2026-10-06T12:00:00Z"
        }
        d = {"lat": 0.0009, "lon": 0.0, "ts": "2026-10-06T12:00:10Z"}
        derived = subscriberJsonOsmAnd.derive_speed_if_needed(d)

        self.assertIsNotNone(derived.get("speed"))
        self.assertEqual(derived.get("speed_source"), "derived")
        self.assertAlmostEqual(derived["speed"], 19.45, places=2)

    @patch("subscriberJsonOsmAnd.requests.get")
    def test_send_osmand_keeps_gnss_knots_as_is(self, mock_get):
        class MockResponse:
            status_code = 200
            text = "OK"
        mock_get.return_value = MockResponse()

        subscriberJsonOsmAnd.send_osmand(0.0, 0.0, speed=10.0)
        args, kwargs = mock_get.call_args
        self.assertEqual(kwargs["params"]["speed"], "10.0")

if __name__ == "__main__":
    unittest.main()
