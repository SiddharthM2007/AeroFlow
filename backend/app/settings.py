"""Runtime configuration, read from environment variables."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Settings:
    engine_path: Path
    data_dir: Path
    engine_timeout_s: float
    max_upload_bytes: int
    max_cells: int
    max_cells_per_axis: int
    max_triangles: int
    max_field_bytes: int
    max_fields_disk_bytes: int
    max_jobs_retained: int
    cors_origins: tuple[str, ...]

    @property
    def models_dir(self) -> Path:
        return self.data_dir / "models"

    @property
    def jobs_dir(self) -> Path:
        """Per-job numerical field files (AFFIELD), kept while the job is retained."""
        return self.data_dir / "jobs"

    @property
    def work_dir(self) -> Path:
        """Scratch space for engine input files (world-space triangle files)."""
        return self.data_dir / "work"


def load_settings() -> Settings:
    env = os.environ
    return Settings(
        engine_path=Path(
            env.get("AEROFLOW_ENGINE_PATH", REPO_ROOT / "engine" / "build" / "aeroflow-grid")
        ),
        data_dir=Path(env.get("AEROFLOW_DATA_DIR", REPO_ROOT / "backend" / "data")),
        engine_timeout_s=float(env.get("AEROFLOW_ENGINE_TIMEOUT_S", "60")),
        max_upload_bytes=int(env.get("AEROFLOW_MAX_UPLOAD_MB", "100")) * 1024 * 1024,
        # 128^3 keeps the base64 classification payload around 2.8 MB.
        max_cells=int(env.get("AEROFLOW_MAX_CELLS", str(128**3))),
        max_cells_per_axis=int(env.get("AEROFLOW_MAX_CELLS_PER_AXIS", "512")),
        max_triangles=int(env.get("AEROFLOW_MAX_TRIANGLES", "1000000")),
        max_field_bytes=int(env.get("AEROFLOW_MAX_FIELD_MB", "64")) * 1024 * 1024,
        max_fields_disk_bytes=int(env.get("AEROFLOW_MAX_FIELDS_DISK_MB", "1024")) * 1024 * 1024,
        max_jobs_retained=int(env.get("AEROFLOW_MAX_JOBS", "50")),
        cors_origins=tuple(
            o.strip()
            for o in env.get(
                "AEROFLOW_CORS_ORIGINS", "http://localhost:5173,http://127.0.0.1:5173"
            ).split(",")
            if o.strip()
        ),
    )
