"""Identifier rules shared by the SX1 parser, the WebSocket routes, and the REST API."""
from __future__ import annotations

import re
from typing import Final

ROBOT_ID_PATTERN: Final = r"[A-Za-z0-9_-]{1,32}"
OPERATOR_ID_PATTERN: Final = r"[A-Za-z0-9_.@-]{1,64}"

ROBOT_ID_RE: Final = re.compile(ROBOT_ID_PATTERN)
OPERATOR_ID_RE: Final = re.compile(OPERATOR_ID_PATTERN)

# Anchored variants for pydantic ``Field(pattern=...)``, which uses search semantics.
ROBOT_ID_FIELD: Final = f"^{ROBOT_ID_PATTERN}$"
OPERATOR_ID_FIELD: Final = f"^{OPERATOR_ID_PATTERN}$"
