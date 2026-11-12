from __future__ import annotations

import math

import pytest
from fastapi.testclient import TestClient

from app.transform import UNIT_TO_METERS, apply_matrix, compose_matrix, rotation_matrix

OBJ = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"

# Same fixture as frontend/src/lib/transform.test.ts; both sides must agree.
FIXTURE = {
    "units": "mm",
    "unit_scale": 0.001,
    "pivot": [10.0, 20.0, 30.0],
    "scale": [2.0, 1.0, 1.0],
    "rotation_deg": [90.0, 0.0, 0.0],
    "position": [1.0, 2.0, 3.0],
}
EXPECTED_ROW_MAJOR = [0.002, 0, 0, 0.98, 0, 0, -0.001, 2.03, 0, 0.001, 0, 2.98, 0, 0, 0, 1]


def _matrix(spec: dict) -> list[float]:
    return compose_matrix(
        spec["unit_scale"], spec["pivot"], spec["scale"], spec["rotation_deg"], spec["position"]
    )


def _transform(**overrides) -> dict:
    spec = {**FIXTURE, **overrides}
    spec.setdefault("matrix", _matrix(spec))
    return spec


# --- reference math ----------------------------------------------------------


def test_fixture_matrix_matches_frontend() -> None:
    for got, want in zip(_matrix(FIXTURE), EXPECTED_ROW_MAJOR):
        assert got == pytest.approx(want, abs=1e-15)


def test_millimeters_to_meters() -> None:
    m = compose_matrix(UNIT_TO_METERS["mm"], [0, 0, 0], [1, 1, 1], [0, 0, 0], [0, 0, 0])
    assert apply_matrix(m, (1000, 250, -40)) == pytest.approx((1.0, 0.25, -0.04))


def test_pivot_maps_to_position() -> None:
    m = _matrix(FIXTURE)
    assert apply_matrix(m, FIXTURE["pivot"]) == pytest.approx(tuple(FIXTURE["position"]))


def test_euler_xyz_order_is_rx_ry_rz() -> None:
    R = rotation_matrix([90, 90, 0])
    # Rx(90)·Ry(90) applied to +x: Ry sends x → −z, then Rx sends −z → +y.
    v = [sum(R[i][j] * [1, 0, 0][j] for j in range(3)) for i in range(3)]
    assert v == pytest.approx([0, 1, 0], abs=1e-12)


def test_rotation_preserves_lengths() -> None:
    R = rotation_matrix([17, -42, 133])
    for col in range(3):
        assert math.hypot(*(R[row][col] for row in range(3))) == pytest.approx(1.0)


# --- API -----------------------------------------------------------------------


def test_limits_endpoint(client: TestClient, settings) -> None:
    body = client.get("/api/limits").json()
    assert body == {
        "max_cells": settings.max_cells,
        "max_cells_per_axis": settings.max_cells_per_axis,
        "max_upload_mb": settings.max_upload_bytes // (1024 * 1024),
        "max_triangles": settings.max_triangles,
        "max_field_mb": settings.max_field_bytes // (1024 * 1024),
    }


@pytest.fixture
def model_id(client: TestClient) -> str:
    return client.post("/api/models", files={"file": ("m.obj", OBJ)}).json()["model_id"]


def test_grid_job_records_model_transform(client, valid_job, model_id, engine_built) -> None:
    valid_job.update(model_id=model_id, model_transform=_transform())
    resp = client.post("/api/grid-jobs", json=valid_job)
    assert resp.status_code == 201, resp.text
    job = resp.json()
    assert job["status"] == "completed"
    stored = job["request"]["model_transform"]
    assert stored["units"] == "mm"
    assert stored["matrix"] == pytest.approx(EXPECTED_ROW_MAJOR, abs=1e-15)
    # Procedural mode: the transform is recorded, but only the test box is classified.
    assert job["result"]["grid"]["cell_count"] == 64


def _errors(resp) -> dict[str, str]:
    assert resp.status_code == 422, resp.text
    return {e["field"]: e["message"] for e in resp.json()["errors"]}


@pytest.mark.parametrize(
    ("overrides", "fragment"),
    [
        ({"unit_scale": 1.0}, "does not match units"),
        ({"units": "ft"}, ""),
        ({"matrix": [1.0] * 16}, "inconsistent with the transform parameters"),
        ({"matrix": [0.0] * 15}, ""),
        ({"scale": [1.0, 0.0, 1.0]}, ""),
        ({"scale": [1.0, -2.0, 1.0]}, ""),
    ],
)
def test_model_transform_validation(client, valid_job, model_id, overrides, fragment) -> None:
    # The matrix stays the fixture's unless overridden, so parameter changes make it inconsistent.
    spec = {**FIXTURE, "matrix": EXPECTED_ROW_MAJOR, **overrides}
    valid_job.update(model_id=model_id, model_transform=spec)
    errors = _errors(client.post("/api/grid-jobs", json=valid_job))
    assert any(f.startswith("model_transform") for f in errors)
    assert any(fragment in m for m in errors.values())


def test_model_transform_requires_model_id(client, valid_job) -> None:
    valid_job["model_transform"] = _transform()
    errors = _errors(client.post("/api/grid-jobs", json=valid_job))
    assert errors["model_transform"] == "requires model_id"


def test_matrix_tolerance_scales_with_magnitude(client, valid_job, model_id, engine_built) -> None:
    # Large translations (e.g. a 100 m tunnel) must not fail on float32-level rounding.
    spec = _transform(position=[100.0, 50.0, 25.0])
    spec["matrix"] = [v * (1 + 1e-12) for v in spec["matrix"]]
    valid_job.update(model_id=model_id, model_transform=spec)
    assert client.post("/api/grid-jobs", json=valid_job).status_code == 201
