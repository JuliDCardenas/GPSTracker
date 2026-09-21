import json
import sqlite3
from datetime import datetime, timezone


class EventStore:
    """Durable inbox for idempotent ignition events and tracker state."""

    def __init__(self, path: str):
        self.path = path
        self.db = sqlite3.connect(path)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.executescript(
            """
            CREATE TABLE IF NOT EXISTS ignition_events (
                event_id TEXT PRIMARY KEY,
                payload_json TEXT NOT NULL,
                status TEXT NOT NULL DEFAULT 'pending',
                received_at TEXT NOT NULL,
                delivered_at TEXT
            );
            CREATE TABLE IF NOT EXISTS runtime_state (
                key TEXT PRIMARY KEY,
                value_json TEXT NOT NULL,
                updated_at TEXT NOT NULL
            );
            """
        )
        self.db.commit()

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def accept_event(self, payload: dict):
        event_id = str(payload["event_id"])
        cur = self.db.execute(
            """
            INSERT OR IGNORE INTO ignition_events
                (event_id, payload_json, status, received_at)
            VALUES (?, ?, 'pending', ?)
            """,
            (event_id, json.dumps(payload, separators=(",", ":")), self._now()),
        )
        self.db.commit()
        row = self.db.execute(
            "SELECT status FROM ignition_events WHERE event_id = ?", (event_id,)
        ).fetchone()
        return cur.rowcount == 1, row["status"]

    def pending_events(self):
        rows = self.db.execute(
            """
            SELECT event_id, payload_json
            FROM ignition_events
            WHERE status = 'pending'
            ORDER BY received_at, rowid
            """
        ).fetchall()
        return [(row["event_id"], json.loads(row["payload_json"])) for row in rows]

    def mark_delivered(self, event_id: str):
        self.db.execute(
            """
            UPDATE ignition_events
            SET status = 'delivered', delivered_at = ?
            WHERE event_id = ?
            """,
            (self._now(), event_id),
        )
        self.db.commit()

    def event_status(self, event_id: str):
        row = self.db.execute(
            "SELECT status FROM ignition_events WHERE event_id = ?", (event_id,)
        ).fetchone()
        return row["status"] if row else None

    def save_state(self, key: str, value):
        self.db.execute(
            """
            INSERT INTO runtime_state (key, value_json, updated_at)
            VALUES (?, ?, ?)
            ON CONFLICT(key) DO UPDATE SET
                value_json = excluded.value_json,
                updated_at = excluded.updated_at
            """,
            (key, json.dumps(value, separators=(",", ":")), self._now()),
        )
        self.db.commit()

    def load_state(self, key: str, default=None):
        row = self.db.execute(
            "SELECT value_json FROM runtime_state WHERE key = ?", (key,)
        ).fetchone()
        return json.loads(row["value_json"]) if row else default

    def close(self):
        self.db.close()
