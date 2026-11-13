"""Thin subprocess wrapper around the standalone `aeroflow-grid` executable."""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from collections.abc import Callable
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


class EngineCancelled(RuntimeError):
    """The run was cancelled by the user."""


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

    def run_streaming(
        self,
        config: dict[str, Any],
        include_cells: bool,
        on_progress: Callable[[dict[str, Any]], None],
        hard_timeout_s: float,
        cancel: threading.Event,
    ) -> dict[str, Any]:
        """Runs the engine, forwarding {"progress": ...} stderr lines as they arrive.

        Kills the process on cancel (EngineCancelled) or after hard_timeout_s (EngineError
        "timeout"); the engine normally stops itself earlier at its own wall-time budget.
        """
        self._require()
        args = [str(self.path), "-"]
        if not include_cells:
            args.append("--no-cells")
        proc = subprocess.Popen(
            args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        stdout_chunks: list[str] = []
        reader = threading.Thread(target=lambda: stdout_chunks.append(proc.stdout.read()), daemon=True)
        reader.start()
        killed: dict[str, str | None] = {"reason": None}
        finished = threading.Event()

        def watchdog() -> None:
            deadline = time.monotonic() + hard_timeout_s
            while not finished.is_set():
                if cancel.wait(0.1):
                    killed["reason"] = "cancelled"
                    break
                if time.monotonic() > deadline:
                    killed["reason"] = "timeout"
                    break
            if killed["reason"] and proc.poll() is None:
                proc.kill()

        guard = threading.Thread(target=watchdog, daemon=True)
        guard.start()
        try:
            proc.stdin.write(json.dumps(config))
            proc.stdin.close()
        except BrokenPipeError:
            pass

        last_line = ""
        for line in proc.stderr:
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except ValueError:
                last_line = line
                continue
            if isinstance(message, dict) and "progress" in message:
                on_progress(message["progress"])
            else:
                last_line = line
        proc.wait()
        reader.join()
        finished.set()
        guard.join()

        if killed["reason"] == "cancelled":
            raise EngineCancelled("simulation cancelled")
        if killed["reason"] == "timeout":
            raise EngineError("timeout", f"engine exceeded the hard limit of {hard_timeout_s:g}s and was stopped")
        if proc.returncode != 0:
            code, message = "engine_failed", last_line or "engine exited with an error"
            try:
                err = json.loads(last_line)["error"]
                code, message = err["code"], err["message"]
            except (ValueError, KeyError, TypeError):
                pass
            raise EngineError(code, message)
        try:
            return json.loads(stdout_chunks[0] if stdout_chunks else "")
        except ValueError as exc:
            raise EngineError("bad_output", "engine produced invalid JSON") from exc
