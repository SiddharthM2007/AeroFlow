from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.main import create_app
from app.settings import load_settings


@pytest.fixture
def settings(tmp_path: Path):
    return dataclasses.replace(load_settings(), data_dir=tmp_path / "data")


@pytest.fixture
def client(settings) -> TestClient:
    return TestClient(create_app(settings))


@pytest.fixture
def engine_built(settings) -> None:
    if not settings.engine_path.is_file():
        pytest.skip(f"engine not built at {settings.engine_path}")


@pytest.fixture
def valid_job() -> dict:
    return {
        "domain": {"lx": 4.0, "ly": 4.0, "lz": 4.0},
        "resolution": {"nx": 4, "ny": 4, "nz": 4},
        "test_box": {"min": [1.0, 1.0, 1.0], "max": [3.0, 3.0, 3.0]},
    }
