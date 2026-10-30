"""Black-box compliance log: capped, thread-safe and tamper-evident.

Each event is chained to its predecessor with a SHA-256 hash, so edits or deletions inside the retained
window are detectable with :meth:`AuditStore.verify`. The JSON file is rewritten atomically (temp file and
``os.replace``), debounced by a background thread; critical events can force an immediate flush.
"""
from __future__ import annotations

import contextlib
import hashlib
import json
import logging
import os
import tempfile
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

log = logging.getLogger("sax.audit")

GENESIS_HASH = "0" * 64


def _digest(event: dict[str, Any]) -> str:
    body = {key: value for key, value in event.items() if key != "hash"}
    encoded = json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class AuditStore:
    """Append-only audit trail persisted to a JSON array (``sax_audit.json``)."""

    def __init__(
        self,
        path: str | os.PathLike[str] = "sax_audit.json",
        *,
        max_events: int = 5000,
        flush_interval_s: float = 1.0,
    ) -> None:
        self._path = Path(path)
        self._max_events = max_events
        self._events: deque[dict[str, Any]] = deque(maxlen=max_events)
        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._dirty = threading.Event()
        self._stop = threading.Event()
        self._seq = 0
        self._last_hash = GENESIS_HASH
        self._load()
        self._flusher = threading.Thread(
            target=self._flush_loop, args=(flush_interval_s,), name="sax-audit-flush", daemon=True
        )
        self._flusher.start()

    @property
    def path(self) -> Path:
        return self._path

    def __len__(self) -> int:
        with self._lock:
            return len(self._events)

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            if not isinstance(data, list) or not all(isinstance(item, dict) for item in data):
                raise ValueError("audit file must contain a JSON array of objects")
        except (OSError, ValueError) as exc:
            backup = self._path.with_name(f"{self._path.name}.corrupt-{int(time.time())}")
            log.error("audit log unreadable (%s); moving it to %s and starting fresh", exc, backup)
            with contextlib.suppress(OSError):
                self._path.replace(backup)
            return
        self._events.extend(data[-self._max_events :])
        if self._events:
            last = self._events[-1]
            self._seq = int(last.get("seq", len(self._events)))
            self._last_hash = str(last.get("hash", GENESIS_HASH))

    def record(
        self,
        event_type: str,
        *,
        operator_id: str | None = None,
        robot_id: str | None = None,
        detail: dict[str, Any] | None = None,
        critical: bool = False,
    ) -> dict[str, Any]:
        """Append an event. ``critical=True`` flushes to disk before returning."""
        now = time.time()
        safe_detail = json.loads(json.dumps(detail or {}, default=str))
        with self._lock:
            self._seq += 1
            event: dict[str, Any] = {
                "seq": self._seq,
                "id": uuid.uuid4().hex,
                "ts": round(now, 3),
                "at": datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="milliseconds"),
                "type": event_type,
                "operator_id": operator_id,
                "robot_id": robot_id,
                "detail": safe_detail,
                "prev_hash": self._last_hash,
            }
            event["hash"] = _digest(event)
            self._last_hash = event["hash"]
            self._events.append(event)
            self._dirty.set()
        if critical:
            self.flush()
        return event

    def snapshot(
        self,
        *,
        limit: int | None = None,
        event_type: str | None = None,
        robot_id: str | None = None,
        since: float | None = None,
    ) -> list[dict[str, Any]]:
        """Return copies of the retained events, oldest first, optionally filtered."""
        with self._lock:
            events = [
                dict(event)
                for event in self._events
                if (event_type is None or event["type"] == event_type)
                and (robot_id is None or event["robot_id"] == robot_id)
                and (since is None or event["ts"] >= since)
            ]
        return events[-limit:] if limit else events

    def verify(self) -> bool:
        """Check every retained event's hash and its link to the previous event."""
        with self._lock:
            previous: str | None = None
            for event in self._events:
                if event.get("hash") != _digest(event):
                    return False
                if previous is not None and event.get("prev_hash") != previous:
                    return False
                previous = event["hash"]
        return True

    def flush(self) -> None:
        """Atomically persist the retained events."""
        with self._io_lock:
            with self._lock:
                data = list(self._events)
                self._dirty.clear()
            tmp_name: str | None = None
            try:
                self._path.parent.mkdir(parents=True, exist_ok=True)
                fd, tmp_name = tempfile.mkstemp(dir=self._path.parent, prefix=f"{self._path.name}.", suffix=".tmp")
                with os.fdopen(fd, "w", encoding="utf-8") as handle:
                    json.dump(data, handle, separators=(",", ":"))
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(tmp_name, self._path)
                tmp_name = None
            except OSError:
                log.exception("failed to persist the audit log")
                self._dirty.set()
            finally:
                if tmp_name is not None:
                    with contextlib.suppress(OSError):
                        os.unlink(tmp_name)

    def _flush_loop(self, interval_s: float) -> None:
        while not self._stop.wait(interval_s):
            if self._dirty.is_set():
                self.flush()

    def close(self) -> None:
        """Stop the flusher thread and write any pending events."""
        self._stop.set()
        self._flusher.join(timeout=2.0)
        self.flush()
