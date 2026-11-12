"""Milestone 4A: AFFIELD validation, per-job storage and field delivery."""

from __future__ import annotations

import dataclasses
import struct
from array import array
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.fields import FIELD_MAGIC, FieldFileError, estimate_field_bytes, parse_field_header
from app.main import create_app

GOLDEN = Path(__file__).resolve().parents[2] / "fixtures" / "fields" / "golden_v1.affield"


def decode(data: bytes) -> tuple[dict, array, bytes]:
    """Minimal reader used by the tests: header, float32 values, validity bytes."""
    header = parse_field_header(data)
    blocks = {b["name"]: b for b in header["blocks"]}
    values = array("f")
    values.frombytes(data[blocks["values"]["offset"] : blocks["values"]["offset"] + blocks["values"]["length"]])
    valid = data[blocks["valid"]["offset"] : blocks["valid"]["offset"] + blocks["valid"]["length"]]
    return header, values, valid


# --- format validation (shared golden file written by the C++ engine) -------------------------


def test_golden_file_decodes_x_fastest() -> None:
    header, values, valid = decode(GOLDEN.read_bytes())
    assert header["version"] == 1
    assert header["shape"] == [3, 2, 2]
    assert header["location"] == "cell"
    assert header["grid"]["spacing"] == [0.5, 0.5, 0.25]
    for k in range(2):
        for j in range(2):
            for i in range(3):
                idx = i + 3 * (j + 2 * k)
                if (i, j, k) == (1, 1, 0):
                    assert valid[idx] == 0
                else:
                    assert valid[idx] == 1
                    assert values[idx] == i + 10 * j + 100 * k + 0.25
    assert header["stats"]["valid_count"] == 11
    assert header["stats"]["max"] == 112.25


def _replace(data: bytes, old: bytes, new: bytes) -> bytes:
    assert old in data and len(old) == len(new)
    return data.replace(old, new, 1)


def _patched(data: bytes, offset: int, payload: bytes) -> bytes:
    return data[:offset] + payload + data[offset + len(payload) :]


@pytest.mark.parametrize(
    ("mutate", "fragment"),
    [
        (lambda d: b"XXXXXXX\0" + d[8:], "bad magic"),
        (lambda d: _patched(d, 8, struct.pack("<I", 2)), "unsupported AFFIELD version 2"),
        (lambda d: d[:40], "truncated"),
        (lambda d: d[:-4], "outside the file"),
        # Same-length edits of the compact JSON header keep every offset valid.
        (lambda d: _replace(d, b'"shape":[3,2,2]', b'"shape":[3,2,3]'), "does not match its location"),
        (lambda d: _replace(d, b'"dtype":"float32"', b'"dtype":"float64"'), "wrong type or length"),
        (lambda d: _replace(d, b'"byte_order":"little"', b'"byte_order":"bigend"'), "byte or index order"),
        (lambda d: _replace(d, b'"version":1', b'"version":9'), "wrong format or version"),
    ],
)
def test_invalid_files_are_rejected(mutate, fragment) -> None:
    with pytest.raises(FieldFileError, match=fragment):
        parse_field_header(mutate(GOLDEN.read_bytes()))


def test_grid_mismatch_is_rejected() -> None:
    with pytest.raises(FieldFileError, match="does not match the job grid"):
        parse_field_header(GOLDEN.read_bytes(), {"nx": 4, "ny": 2, "nz": 2})


def test_size_estimate_bounds_real_files() -> None:
    assert GOLDEN.stat().st_size <= estimate_field_bytes(12)


# --- API --------------------------------------------------------------------------------------


def _job(fields=("divergence_diagnostic",), box=True) -> dict:
    job = {
        "domain": {"lx": 2.4, "ly": 1.6, "lz": 1.6},
        "resolution": {"nx": 12, "ny": 8, "nz": 8},  # dx = 0.2
        "fields": list(fields),
        "include_cells": False,
    }
    if box:
        job["test_box"] = {"min": [0.8, 0.6, 0.6], "max": [1.6, 1.0, 1.0]}  # i 4..7, j 3..4, k 3..4
    return job


def test_field_listing_and_binary_delivery(client: TestClient, engine_built, settings) -> None:
    job = client.post("/api/grid-jobs", json=_job()).json()
    assert job["status"] == "completed", job["error"]
    assert [f["name"] for f in job["fields"]] == ["divergence_diagnostic"]
    meta = job["fields"][0]
    assert meta["units"] == "1/s"
    assert meta["provenance"]["kind"] == "diagnostic" and meta["provenance"]["solved"] is False
    assert "blocks" not in meta

    listing = client.get(f"/api/grid-jobs/{job['job_id']}/fields").json()
    assert listing["fields"] == job["fields"]

    resp = client.get(f"/api/grid-jobs/{job['job_id']}/fields/divergence_diagnostic")
    assert resp.status_code == 200
    assert resp.headers["content-type"] == "application/octet-stream"
    assert resp.headers["x-aeroflow-field-version"] == "1"
    assert resp.content[:8] == FIELD_MAGIC
    header, values, valid = decode(resp.content)
    assert header["shape"] == [12, 8, 8]
    assert len(values) == 12 * 8 * 8

    # Divergence is localized at the obstacle's upstream/downstream faces: -/+ 5 m/s / 0.2 m.
    for k in range(8):
        for j in range(8):
            for i in range(12):
                idx = i + 12 * (j + 8 * k)
                shadow = 3 <= j <= 4 and 3 <= k <= 4
                if shadow and 4 <= i <= 7:
                    assert valid[idx] == 0
                elif shadow and i == 3:
                    assert values[idx] == pytest.approx(-25.0)
                elif shadow and i == 8:
                    assert values[idx] == pytest.approx(25.0)
                else:
                    assert valid[idx] == 1 and values[idx] == 0.0
    assert meta["stats"]["max_abs"] == pytest.approx(25.0)
    assert meta["stats"]["valid_count"] == 12 * 8 * 8 - 4 * 2 * 2


def test_unobstructed_field_is_divergence_free(client: TestClient, engine_built) -> None:
    job = client.post("/api/grid-jobs", json=_job(box=False)).json()
    meta = job["fields"][0]
    assert meta["stats"]["max_abs"] == 0.0
    assert meta["stats"]["rms"] == 0.0
    _, values, _ = decode(client.get(f"/api/grid-jobs/{job['job_id']}/fields/divergence_diagnostic").content)
    assert max(abs(v) for v in values) == 0.0


def test_fields_are_opt_in(client: TestClient, engine_built, settings) -> None:
    job = client.post("/api/grid-jobs", json=_job(fields=())).json()
    assert job["fields"] == []
    assert not (settings.jobs_dir / job["job_id"]).exists()
    resp = client.get(f"/api/grid-jobs/{job['job_id']}/fields/divergence_diagnostic")
    assert resp.status_code == 404
    assert "available: none" in resp.json()["detail"]


def test_missing_field_and_job_errors(client: TestClient, engine_built) -> None:
    job = client.post("/api/grid-jobs", json=_job()).json()
    jid = job["job_id"]
    assert client.get(f"/api/grid-jobs/{jid}/fields/pressure").status_code == 404
    assert client.get(f"/api/grid-jobs/{jid}/fields/divergence_diagnostic?frame=1").status_code == 404
    assert client.get("/api/grid-jobs/nope/fields").status_code == 404
    assert client.get("/api/grid-jobs/nope/fields/divergence_diagnostic").status_code == 404


def test_deleted_field_data_is_reported_gone(client: TestClient, engine_built, settings) -> None:
    jid = client.post("/api/grid-jobs", json=_job()).json()["job_id"]
    for f in (settings.jobs_dir / jid).iterdir():
        f.unlink()
    resp = client.get(f"/api/grid-jobs/{jid}/fields/divergence_diagnostic")
    assert resp.status_code == 410


@pytest.mark.parametrize(
    ("fields", "fragment"),
    [(["vorticity"], "fields"), (["divergence_diagnostic", "divergence_diagnostic"], "unique")],
)
def test_field_request_validation(client: TestClient, fields, fragment) -> None:
    resp = client.post("/api/grid-jobs", json=_job(fields=fields))
    assert resp.status_code == 422
    assert fragment in str(resp.json()["errors"])


def test_field_payload_limit(settings, engine_built) -> None:
    small = TestClient(create_app(dataclasses.replace(settings, max_field_bytes=70_000)))
    resp = small.post("/api/grid-jobs", json=_job())  # 768 cells: ~68 KB estimate is fine
    assert resp.status_code == 201
    big = _job()
    big["resolution"] = {"nx": 48, "ny": 32, "nz": 32}
    resp = small.post("/api/grid-jobs", json=big)
    assert resp.status_code == 422
    assert "AEROFLOW_MAX_FIELD_MB" in resp.json()["detail"]


def test_disk_budget(settings, engine_built) -> None:
    # One field's estimate (~69 KB) fits; the second does not once the first file (~5 KB) exists.
    tiny = TestClient(create_app(dataclasses.replace(settings, max_fields_disk_bytes=72_000)))
    assert tiny.post("/api/grid-jobs", json=_job()).status_code == 201
    resp = tiny.post("/api/grid-jobs", json=_job())  # first job's files still retained
    assert resp.status_code == 507


def test_evicted_jobs_release_their_fields(settings, engine_built) -> None:
    app = TestClient(create_app(dataclasses.replace(settings, max_jobs_retained=2)))
    ids = [app.post("/api/grid-jobs", json=_job()).json()["job_id"] for _ in range(3)]
    assert not (settings.jobs_dir / ids[0]).exists()
    assert (settings.jobs_dir / ids[2]).exists()
    assert app.get(f"/api/grid-jobs/{ids[0]}/fields").status_code == 404


def test_failed_jobs_leave_no_field_files(client: TestClient, engine_built, settings) -> None:
    open_obj = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\n"
    model_id = client.post("/api/models", files={"file": ("open.obj", open_obj)}).json()["model_id"]
    from app.transform import compose_matrix

    job = _job(box=False)
    job.update(
        geometry_source="mesh",
        model_id=model_id,
        model_transform={
            "units": "m", "unit_scale": 1.0, "pivot": [0, 0, 0], "scale": [1, 1, 1],
            "rotation_deg": [0, 0, 0], "position": [1, 0.5, 0.5],
            "matrix": compose_matrix(1.0, [0, 0, 0], [1, 1, 1], [0, 0, 0], [1, 0.5, 0.5]),
        },
    )
    body = client.post("/api/grid-jobs", json=job).json()
    assert body["status"] == "failed"
    assert body["fields"] == []
    assert not (settings.jobs_dir / body["job_id"]).exists()


def test_orphaned_fields_are_cleared_at_startup(settings) -> None:
    orphan = settings.jobs_dir / "old-job"
    orphan.mkdir(parents=True)
    (orphan / "divergence_diagnostic.f0.affield").write_bytes(b"x")
    create_app(settings)
    assert not orphan.exists()
