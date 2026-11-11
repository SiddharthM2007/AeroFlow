from __future__ import annotations

import base64
import dataclasses
import struct

import pytest
from fastapi.testclient import TestClient

from app.main import create_app


def _binary_stl(triangles: int) -> bytes:
    header = b"solid but actually binary".ljust(80, b" ")
    facet = struct.pack("<12fH", *([0.0] * 12), 0)
    return header + struct.pack("<I", triangles) + facet * triangles


ASCII_STL = b"""solid cube
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 1 0
 endloop
endfacet
endsolid cube
"""

OBJ = b"# tri\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"


# --- health -------------------------------------------------------------------


def test_health_reports_engine(client: TestClient, engine_built) -> None:
    body = client.get("/api/health").json()
    assert body["status"] == "ok"
    assert body["engine"]["available"] is True
    assert body["engine"]["version"].startswith("aeroflow-grid")


def test_health_without_engine(settings, tmp_path) -> None:
    app = create_app(dataclasses.replace(settings, engine_path=tmp_path / "missing"))
    body = TestClient(app).get("/api/health").json()
    assert body["status"] == "ok"
    assert body["engine"]["available"] is False


# --- model upload ---------------------------------------------------------------


@pytest.mark.parametrize(
    ("name", "data", "expected"),
    [
        ("part.stl", _binary_stl(3), {"format": "stl", "encoding": "binary", "triangle_count": 3}),
        ("part.STL", ASCII_STL, {"format": "stl", "encoding": "ascii", "triangle_count": 1}),
        ("part.obj", OBJ, {"format": "obj", "vertex_count": 3, "face_count": 1}),
    ],
)
def test_upload_valid_models(client: TestClient, name, data, expected) -> None:
    resp = client.post("/api/models", files={"file": (name, data)})
    assert resp.status_code == 201, resp.text
    body = resp.json()
    for key, value in expected.items():
        assert body[key] == value
    assert len(body["model_id"]) == 32
    assert client.get(f"/api/models/{body['model_id']}").json()["filename"] == name


@pytest.mark.parametrize(
    ("name", "data", "fragment"),
    [
        ("part.step", b"ISO-10303", "unsupported file type"),
        ("noext", b"data", "unsupported file type"),
        ("part.stl", b"", "file is empty"),
        ("part.stl", b"not an stl at all", "not a valid STL"),
        ("part.stl", _binary_stl(3)[:-10], "not a valid STL"),
        ("part.stl", b"solid empty\nendsolid empty\n", "no facets"),
        ("part.obj", b"# only comments\n", "no vertices"),
        ("part.obj", b"v 0 0 0\nv 1 0 0\n", "no faces"),
        ("part.obj", b"\x00\x01\x02binary", "must be text"),
    ],
)
def test_upload_rejects_invalid_models(client: TestClient, name, data, fragment) -> None:
    resp = client.post("/api/models", files={"file": (name, data)})
    assert resp.status_code == 400
    assert fragment in resp.json()["detail"]


def test_upload_enforces_size_limit(settings) -> None:
    app = create_app(dataclasses.replace(settings, max_upload_bytes=100))
    resp = TestClient(app).post("/api/models", files={"file": ("big.stl", _binary_stl(3))})
    assert resp.status_code == 413


def test_upload_requires_file(client: TestClient) -> None:
    resp = client.post("/api/models")
    assert resp.status_code == 422
    assert resp.json()["errors"][0]["field"] == "file"


def test_unknown_model_404(client: TestClient) -> None:
    assert client.get("/api/models/" + "0" * 32).status_code == 404
    assert client.get("/api/models/../../etc").status_code == 404


# --- grid job validation -------------------------------------------------------


def _fields(resp) -> dict[str, str]:
    assert resp.status_code == 422, resp.text
    return {e["field"]: e["message"] for e in resp.json()["errors"]}


@pytest.mark.parametrize(
    ("patch", "field"),
    [
        (lambda j: j["domain"].update(lx=0), "domain.lx"),
        (lambda j: j["domain"].update(ly=-2.0), "domain.ly"),
        (lambda j: j["resolution"].update(nx=0), "resolution.nx"),
        (lambda j: j["resolution"].update(ny=2.5), "resolution.ny"),
        (lambda j: j["resolution"].update(nz="8"), "resolution.nz"),
        (lambda j: j["resolution"].pop("nx"), "resolution.nx"),
        (lambda j: j.pop("domain"), "domain"),
        (lambda j: j.update(solver="fast"), "solver"),
        (lambda j: j["test_box"].update(min=[1.0, 1.0]), "test_box.min"),
    ],
)
def test_grid_job_field_validation(client: TestClient, valid_job, patch, field) -> None:
    patch(valid_job)
    assert field in _fields(client.post("/api/grid-jobs", json=valid_job))


@pytest.mark.parametrize(
    ("box", "fragment"),
    [
        ({"min": [2.0, 1.0, 1.0], "max": [1.0, 3.0, 3.0]}, "min.x must be less than max.x"),
        ({"min": [1.0, 1.0, 1.0], "max": [3.0, 1.0, 3.0]}, "min.y must be less than max.y"),
        ({"min": [1.0, 1.0, 1.0], "max": [3.0, 3.0, 5.0]}, "extent along z"),
        ({"min": [-0.5, 1.0, 1.0], "max": [3.0, 3.0, 3.0]}, "extent along x"),
    ],
)
def test_grid_job_box_validation(client: TestClient, valid_job, box, fragment) -> None:
    valid_job["test_box"] = box
    fields = _fields(client.post("/api/grid-jobs", json=valid_job))
    assert fragment in fields["test_box"]


def test_grid_job_cell_limit(client: TestClient, valid_job) -> None:
    valid_job["resolution"] = {"nx": 512, "ny": 512, "nz": 512}
    resp = client.post("/api/grid-jobs", json=valid_job)
    assert resp.status_code == 422
    assert "server limit" in resp.json()["detail"]


def test_grid_job_per_axis_limit(client: TestClient, valid_job) -> None:
    valid_job["resolution"] = {"nx": 1000, "ny": 1, "nz": 1}
    resp = client.post("/api/grid-jobs", json=valid_job)
    assert resp.status_code == 422
    assert "resolution.nx" in resp.json()["detail"]


def test_grid_job_unknown_model(client: TestClient, valid_job) -> None:
    valid_job["model_id"] = "f" * 32
    assert client.post("/api/grid-jobs", json=valid_job).status_code == 404


def test_grid_job_engine_missing_is_503(settings, tmp_path, valid_job) -> None:
    app = create_app(dataclasses.replace(settings, engine_path=tmp_path / "missing"))
    resp = TestClient(app).post("/api/grid-jobs", json=valid_job)
    assert resp.status_code == 503
    assert "engine executable not found" in resp.json()["detail"]


# --- grid job end-to-end (requires built engine) --------------------------------


def test_grid_job_runs_engine(client: TestClient, valid_job, engine_built) -> None:
    resp = client.post("/api/grid-jobs", json=valid_job)
    assert resp.status_code == 201, resp.text
    job = resp.json()
    assert job["status"] == "completed"
    result = job["result"]
    assert result["grid"]["cell_count"] == 64
    assert result["grid"]["spacing"] == {"dx": 1.0, "dy": 1.0, "dz": 1.0}
    assert result["classification"]["solid_count"] == 8

    cells = base64.b64decode(result["classification"]["data"])
    assert len(cells) == 64
    nx = ny = 4
    for k in range(4):
        for j in range(4):
            for i in range(4):
                inside = all(1 <= n <= 2 for n in (i, j, k))  # centers 1.5, 2.5 in [1, 3)
                assert cells[i + nx * (j + ny * k)] == (1 if inside else 0)

    fetched = client.get(f"/api/grid-jobs/{job['job_id']}")
    assert fetched.status_code == 200
    assert fetched.json()["result"]["classification"]["solid_count"] == 8


def test_grid_job_without_cells_or_box(client: TestClient, valid_job, engine_built) -> None:
    valid_job.pop("test_box")
    valid_job["include_cells"] = False
    job = client.post("/api/grid-jobs", json=valid_job).json()
    assert job["status"] == "completed"
    assert job["result"]["test_box"] is None
    assert job["result"]["classification"]["solid_count"] == 0
    assert "data" not in job["result"]["classification"]


def test_grid_job_with_uploaded_model(client: TestClient, valid_job, engine_built) -> None:
    model_id = client.post("/api/models", files={"file": ("m.obj", OBJ)}).json()["model_id"]
    valid_job["model_id"] = model_id
    job = client.post("/api/grid-jobs", json=valid_job).json()
    assert job["status"] == "completed"
    assert job["request"]["model_id"] == model_id


def test_unknown_job_404(client: TestClient) -> None:
    assert client.get("/api/grid-jobs/nope").status_code == 404
