import os
import tempfile
import unittest

from event_store import EventStore


class EventStoreTests(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".sqlite3")
        os.close(fd)
        self.store = EventStore(self.path)

    def tearDown(self):
        self.store.close()
        for suffix in ("", "-wal", "-shm"):
            try:
                os.remove(self.path + suffix)
            except FileNotFoundError:
                pass

    def test_duplicate_event_is_inserted_once(self):
        event = {"event_id": "abc-1", "type": "engine_on", "ignition": 1}
        inserted, status = self.store.accept_event(event)
        self.assertTrue(inserted)
        self.assertEqual("pending", status)
        inserted, status = self.store.accept_event(event)
        self.assertFalse(inserted)
        self.assertEqual("pending", status)
        self.assertEqual(1, len(self.store.pending_events()))

    def test_delivered_duplicate_stays_delivered(self):
        event = {"event_id": "abc-2", "type": "engine_off", "ignition": 0}
        self.store.accept_event(event)
        self.store.mark_delivered("abc-2")
        inserted, status = self.store.accept_event(event)
        self.assertFalse(inserted)
        self.assertEqual("delivered", status)
        self.assertEqual([], self.store.pending_events())

    def test_last_point_survives_reopen(self):
        point = {"lat": 4.6, "lon": -74.1, "ts": "2026-09-18T20:00:00Z"}
        self.store.save_state("last_valid_point", point)
        self.store.close()
        self.store = EventStore(self.path)
        self.assertEqual(point, self.store.load_state("last_valid_point"))


if __name__ == "__main__":
    unittest.main()
