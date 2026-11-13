"""In-memory job registry (no database in Milestone 1)."""

from __future__ import annotations

import threading
from collections import OrderedDict
from collections.abc import Callable

from .schemas import FINAL_STATUSES, GridJob


class JobStore:
    def __init__(self, max_jobs: int, on_evict: Callable[[str], None] | None = None) -> None:
        self._jobs: OrderedDict[str, GridJob] = OrderedDict()
        self._max = max_jobs
        self._lock = threading.Lock()
        self._on_evict = on_evict

    def add(self, job: GridJob) -> None:
        evicted = []
        with self._lock:
            self._jobs[job.job_id] = job
            # Evict the oldest finished jobs only; queued/running simulations are never evicted.
            for job_id in list(self._jobs):
                if len(self._jobs) - len(evicted) <= self._max:
                    break
                if self._jobs[job_id].status in FINAL_STATUSES and job_id != job.job_id:
                    evicted.append(job_id)
            for job_id in evicted:
                del self._jobs[job_id]
        if self._on_evict:
            for job_id in evicted:
                self._on_evict(job_id)  # e.g. delete the job's field files

    def get(self, job_id: str) -> GridJob | None:
        with self._lock:
            return self._jobs.get(job_id)

    def update(self, job_id: str, **changes: object) -> GridJob | None:
        """Atomically replaces fields of a stored job (used by the simulation worker)."""
        with self._lock:
            job = self._jobs.get(job_id)
            if job is None:
                return None
            job = job.model_copy(update=changes)
            self._jobs[job_id] = job
            return job
