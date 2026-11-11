"""Thin subprocess wrapper around the standalone `aeroflow-grid` executable."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any


class EngineUnavailable(RuntimeError):
    """The engine executable is missing or not executable."""


class EngineError(RuntimeError):
    """The engine ran but reported a failure."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class GridEngine:
    def __init__(self, path: Path, timeout_s: float) -> None:
        self.path = path
        self.timeout_s = timeout_s

    def available(self) -> bool:
        return self.path.is_file() and os.access(self.path, os.X_OK)

    def _require(self) -> None:
        if not self.available():
            raise EngineUnavailable(
                f"engine executable not found at {self.path}; build it with "
                "`cmake -S engine -B engine/build && cmake --build engine/build` "
                "or set AEROFLOW_ENGINE_PATH"
            )

    def version(self) -> str:
        self._require()
        proc = subprocess.run(
            [str(self.path), "--version"], capture_output=True, text=True, timeout=10
        )
        return proc.stdout.strip()

    def generate(self, config: dict[str, Any], include_cells: bool) -> dict[str, Any]:
        self._require()
        args = [str(self.path), "-"]
        if not include_cells:
            args.append("--no-cells")
        try:
            proc = subprocess.run(
                args,
                input=json.dumps(config),
                capture_output=True,
                text=True,
                timeout=self.timeout_s,
            )
        except subprocess.TimeoutExpired as exc:
            raise EngineError("timeout", f"engine exceeded {self.timeout_s:g}s timeout") from exc

        if proc.returncode != 0:
            code, message = "engine_failed", proc.stderr.strip() or "engine exited with an error"
            try:
                err = json.loads(proc.stderr.strip().splitlines()[-1])["error"]
                code, message = err["code"], err["message"]
            except (ValueError, KeyError, IndexError, TypeError):
                pass
            raise EngineError(code, message)

        try:
            return json.loads(proc.stdout)
        except ValueError as exc:
            raise EngineError("bad_output", "engine produced invalid JSON") from exc
