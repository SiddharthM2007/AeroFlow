"""In-memory job registry (no database in Milestone 1)."""

from __future__ import annotations

import threading
from collections import OrderedDict

from .schemas import GridJob


class JobStore:
    def __init__(self, max_jobs: int) -> None:
        self._jobs: OrderedDict[str, GridJob] = OrderedDict()
        self._max = max_jobs
        self._lock = threading.Lock()

    def add(self, job: GridJob) -> None:
        with self._lock:
            self._jobs[job.job_id] = job
            while len(self._jobs) > self._max:
                self._jobs.popitem(last=False)

    def get(self, job_id: str) -> GridJob | None:
        with self._lock:
            return self._jobs.get(job_id)
