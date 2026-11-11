"""Milestone 2B: stored model -> world-space triangles -> C++ voxelization."""

from __future__ import annotations

import base64
import struct
from array import array
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.mesh_io import (
    TRIANGLE_FILE_MAGIC,
    MeshParseError,
    bounds,
    parse_obj,
    parse_stl,
    prepare_world_mesh,
    transform_positions,
)
from app.transform import compose_matrix

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "geometry" / "box_quads.obj"

# Placement shared with frontend/src/lib/consistency.test.ts.
TRANSFORM = {
    "units": "cm",
    "unit_scale": 0.01,
    "pivot": [20.0, 10.0, 5.0],
    "scale": [1.0, 2.0, 1.0],
    "rotation_deg": [0.0, 90.0, 0.0],
    "position": [1.0, 1.0, 1.0],
}
EXPECTED_MIN = (0.95, 0.8, 0.8)
EXPECTED_MAX = (1.05, 1.2, 1.2)


def _matrix() -> list[float]:
    t = TRANSFORM
    return compose_matrix(t["unit_scale"], t["pivot"], t["scale"], t["rotation_deg"], t["position"])


def _box_triangles() -> list[tuple[float, ...]]:
    raw = parse_obj(FIXTURE.read_bytes())
    return [tuple(raw[n : n + 9]) for n in range(0, len(raw), 9)]


def _binary_stl(triangles) -> bytes:
    out = bytearray(b"binary".ljust(80, b" ") + struct.pack("<I", len(triangles)))
    for t in triangles:
        out += struct.pack("<12fH", 0, 0, 0, *t, 0)
    return bytes(out)


def _ascii_stl(triangles) -> bytes:
    lines = ["solid box"]
    for t in triangles:
        lines += ["facet normal 0 0 0", "outer loop"]
        lines += [f"vertex {t[i]} {t[i + 1]} {t[i + 2]}" for i in (0, 3, 6)]
        lines += ["endloop", "endfacet"]
    return ("\n".join(lines + ["endsolid box"]) + "\n").encode()


# --- parsing (must match Three.js) --------------------------------------------------------


def test_obj_fan_triangulation_and_negative_indices() -> None:
    tris = _box_triangles()
    assert len(tris) == 12  # 6 quads -> 2 triangles each
    # Face "f 1 4 3 2" -> (1,4,3), (1,3,2)
    assert tris[0] == (0, 0, 0, 0, 20, 0, 40, 20, 0)
    assert tris[1] == (0, 0, 0, 40, 20, 0, 40, 0, 0)
    # Negative indices: "f -8 -4 -1 -5" == "f 1 5 8 4"
    assert tris[8] == (0, 0, 0, 0, 0, 10, 0, 20, 10)


def test_stl_binary_and_ascii_match_obj() -> None:
    tris = _box_triangles()
    flat = [c for t in tris for c in t]
    assert list(parse_stl(_binary_stl(tris))) == flat
    assert list(parse_stl(_ascii_stl(tris))) == flat


def test_raw_coordinates_are_rounded_to_float32_like_the_viewer() -> None:
    raw = parse_obj(b"v 0.1 0.2 0.3\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    assert raw.typecode == "f"
    assert raw[0] == struct.unpack("<f", struct.pack("<f", 0.1))[0] != 0.1


@pytest.mark.parametrize(
    ("data", "fragment"),
    [
        (b"v 0 0 0\nv 1 0 0\nf 1 2 5\n", "out of range"),
        (b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n", "out of range"),
        (b"v 0 0\n", "three coordinates"),
        (b"v 0 0 0\nf a b c\n", "invalid face index"),
    ],
)
def test_obj_errors(data: bytes, fragment: str) -> None:
    with pytest.raises(MeshParseError, match=fragment):
        parse_obj(data)


def test_ascii_stl_with_incomplete_facet() -> None:
    with pytest.raises(MeshParseError, match="three vertices"):
        parse_stl(b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\n")


# --- transform & engine file ------------------------------------------------------------


def test_world_bounds_match_the_frontend() -> None:
    world = transform_positions(parse_obj(FIXTURE.read_bytes()), _matrix())
    lo, hi = bounds(world)
    assert lo == pytest.approx(EXPECTED_MIN, abs=1e-12)
    assert hi == pytest.approx(EXPECTED_MAX, abs=1e-12)


def test_millimeters_to_meters_in_world_file() -> None:
    m = compose_matrix(0.001, [0, 0, 0], [1, 1, 1], [0, 0, 0], [0, 0, 0])
    world = transform_positions(array("f", [1000, 2000, -500, 0, 0, 0, 1, 1, 1]), m)
    assert list(world[:3]) == pytest.approx([1.0, 2.0, -0.5])


def test_triangle_file_format(tmp_path: Path) -> None:
    out = tmp_path / "w.aftri"
    count = prepare_world_mesh(FIXTURE, "obj", _matrix(), 100, out)
    data = out.read_bytes()
    assert count == 12
    assert data[:8] == TRIANGLE_FILE_MAGIC
    assert struct.unpack_from("<Q", data, 8)[0] == 12
    assert len(data) == 16 + 12 * 72
    world = struct.unpack_from("<108d", data, 16)
    lo, hi = bounds(world)
    assert lo == pytest.approx(EXPECTED_MIN, abs=1e-12)
    assert hi == pytest.approx(EXPECTED_MAX, abs=1e-12)


def test_triangle_limit(tmp_path: Path) -> None:
    with pytest.raises(MeshParseError, match="12 triangles; the server limit is 10"):
        prepare_world_mesh(FIXTURE, "obj", _matrix(), 10, tmp_path / "w.aftri")


# --- API ---------------------------------------------------------------------------------


@pytest.fixture
def box_model(client: TestClient) -> str:
    resp = client.post("/api/models", files={"file": ("box_quads.obj", FIXTURE.read_bytes())})
    assert resp.status_code == 201, resp.text
    return resp.json()["model_id"]


def _mesh_job(model_id: str, n: int = 40) -> dict:
    return {
        "domain": {"lx": 2.0, "ly": 2.0, "lz": 2.0},
        "resolution": {"nx": n, "ny": n, "nz": n},
        "geometry_source": "mesh",
        "model_id": model_id,
        "model_transform": {**TRANSFORM, "matrix": _matrix()},
    }


def test_mesh_job_produces_expected_mask(client, box_model, engine_built, settings) -> None:
    resp = client.post("/api/grid-jobs", json=_mesh_job(box_model))
    assert resp.status_code == 201, resp.text
    job = resp.json()
    assert job["status"] == "completed", job["error"]
    result = job["result"]
    assert result["geometry_source"] == "mesh"
    assert result["test_box"] is None
    # dx = 0.05: x centers 0.975, 1.025 lie in (0.95, 1.05); y and z centers 0.825..1.175 in (0.8, 1.2).
    assert result["classification"]["solid_count"] == 2 * 8 * 8
    mesh = result["mesh"]
    assert mesh["triangles"] == 12
    assert mesh["bounds"]["min"] == pytest.approx(EXPECTED_MIN, abs=1e-12)
    assert mesh["bounds"]["max"] == pytest.approx(EXPECTED_MAX, abs=1e-12)
    assert mesh["volume"] == pytest.approx(0.016, rel=1e-12)  # 0.1 x 0.4 x 0.4 m
    # 128 cells x 0.05^3 = 0.016 m^3: the box is aligned with cell faces, so the error is ~0.
    assert mesh["volume_relative_error"] == pytest.approx(0.0, abs=1e-9)
    assert result["voxelization"]["ray_triangle_tests"] > 0

    cells = base64.b64decode(result["classification"]["data"])
    n = 40
    for k in range(n):
        for j in range(n):
            for i in range(n):
                expect = i in (19, 20) and 16 <= j <= 23 and 16 <= k <= 23
                assert cells[i + n * (j + n * k)] == (1 if expect else 0), (i, j, k)
    # The temporary world-space file is removed after the job.
    assert not any(settings.work_dir.glob("*.aftri"))


def test_mesh_job_is_repeatable(client, box_model, engine_built) -> None:
    first = client.post("/api/grid-jobs", json=_mesh_job(box_model)).json()
    second = client.post("/api/grid-jobs", json=_mesh_job(box_model)).json()
    assert first["result"]["classification"]["data"] == second["result"]["classification"]["data"]


def test_procedural_mode_still_works(client, valid_job, engine_built) -> None:
    job = client.post("/api/grid-jobs", json=valid_job).json()
    assert job["result"]["geometry_source"] == "procedural_box"
    assert job["result"]["classification"]["solid_count"] == 8


def _errors(resp) -> dict[str, str]:
    assert resp.status_code == 422, resp.text
    return {e["field"]: e["message"] for e in resp.json()["errors"]}


def test_mesh_mode_rejects_a_test_box(client, box_model) -> None:
    job = _mesh_job(box_model)
    job["test_box"] = {"min": [0.1, 0.1, 0.1], "max": [0.2, 0.2, 0.2]}
    assert "never combined" in _errors(client.post("/api/grid-jobs", json=job))["test_box"]


@pytest.mark.parametrize("missing", ["model_id", "model_transform"])
def test_mesh_mode_requires_model_and_transform(client, box_model, missing) -> None:
    job = _mesh_job(box_model)
    job.pop(missing)
    errors = _errors(client.post("/api/grid-jobs", json=job))
    assert any(missing in f"{field} {message}" for field, message in errors.items())


def test_open_mesh_error_propagates(client, engine_built) -> None:
    open_obj = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\n"  # tetra minus a face
    model_id = client.post("/api/models", files={"file": ("open.obj", open_obj)}).json()["model_id"]
    job = _mesh_job(model_id)
    job["model_transform"] = {
        "units": "m",
        "unit_scale": 1.0,
        "pivot": [0, 0, 0],
        "scale": [1, 1, 1],
        "rotation_deg": [0, 0, 0],
        "position": [0.5, 0.5, 0.5],
        "matrix": compose_matrix(1.0, [0, 0, 0], [1, 1, 1], [0, 0, 0], [0.5, 0.5, 0.5]),
    }
    resp = client.post("/api/grid-jobs", json=job)
    assert resp.status_code == 201
    body = resp.json()
    assert body["status"] == "failed"
    assert body["error"]["code"] == "invalid_mesh"
    assert "not watertight" in body["error"]["message"]


def test_triangle_limit_is_enforced(client, box_model, settings, engine_built) -> None:
    import dataclasses

    from app.main import create_app

    small = TestClient(create_app(dataclasses.replace(settings, max_triangles=5)))
    model_id = small.post("/api/models", files={"file": ("b.obj", FIXTURE.read_bytes())}).json()["model_id"]
    resp = small.post("/api/grid-jobs", json=_mesh_job(model_id))
    assert resp.status_code == 422
    assert "server limit is 5" in resp.json()["detail"]
