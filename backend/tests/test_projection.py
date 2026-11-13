"""Milestone 5: pressure projection fields through the job API."""

from __future__ import annotations

from array import array

import pytest
from fastapi.testclient import TestClient

from app.fields import parse_field_header

PROJECTION_FIELDS = ["pressure", "divergence_before", "divergence_after", "velocity_magnitude"]


def _decode(data: bytes) -> tuple[dict, array, bytes]:
    header = parse_field_header(data)
    blocks = {b["name"]: b for b in header["blocks"]}
    values = array("f")
    values.frombytes(data[blocks["values"]["offset"] : blocks["values"]["offset"] + blocks["values"]["length"]])
    valid = data[blocks["valid"]["offset"] : blocks["valid"]["offset"] + blocks["valid"]["length"]]
    return header, values, valid


def _job(fields=PROJECTION_FIELDS, box=True, **projection) -> dict:
    job = {
        "domain": {"lx": 2.4, "ly": 1.2, "lz": 1.2},
        "resolution": {"nx": 24, "ny": 12, "nz": 12},  # h = 0.1
        "fields": list(fields),
        "include_cells": False,
    }
    if box:
        job["test_box"] = {"min": [0.8, 0.3, 0.3], "max": [1.2, 0.9, 0.9]}
    if projection:
        job["projection"] = projection
    return job


def test_projection_fields_and_statistics(client: TestClient, engine_built) -> None:
    job = client.post("/api/grid-jobs", json=_job()).json()
    assert job["status"] == "completed", job["error"]
    assert [f["name"] for f in job["fields"]] == PROJECTION_FIELDS
    proj = job["result"]["projection"]
    assert proj["converged"] is True
    assert proj["relative_residual"] <= 1e-10
    assert proj["dt_source"] == "cfl_0.5"
    assert proj["dt"] == pytest.approx(0.5 * 0.1 / 5.0)
    assert proj["fluxes"]["wall_leak"] == 0.0
    assert proj["fluxes"]["outlet"] == pytest.approx(proj["fluxes"]["inlet"], rel=1e-8)
    assert proj["divergence_before"]["max_abs"] == pytest.approx(5.0 / 0.1)
    assert proj["divergence_after"]["max_abs"] <= 1e-8 * 5.0 / 0.1

    units = {f["name"]: (f["units"], f["signed"]) for f in job["fields"]}
    assert units == {
        "pressure": ("Pa", True),
        "divergence_before": ("1/s", True),
        "divergence_after": ("1/s", True),
        "velocity_magnitude": ("m/s", False),
    }
    for f in job["fields"]:
        assert f["provenance"]["kind"] == "projection" and f["provenance"]["solved"] is False

    header, p, valid = _decode(client.get(f"/api/grid-jobs/{job['job_id']}/fields/pressure").content)
    assert header["shape"] == [24, 12, 12]
    # Stagnation upstream of the obstacle: positive pressure in front, solid cells invalid.
    front = 7 + 24 * (6 + 12 * 6)  # cell just upstream of the box (i = 7)
    inside = 10 + 24 * (6 + 12 * 6)
    assert valid[inside] == 0
    assert p[front] > 0


def test_uniform_empty_tunnel_needs_no_correction(client: TestClient, engine_built) -> None:
    job = client.post("/api/grid-jobs", json=_job(box=False)).json()
    proj = job["result"]["projection"]
    assert proj["iterations"] == 0
    assert proj["divergence_after"]["max_abs"] == 0.0
    _, p, _ = _decode(client.get(f"/api/grid-jobs/{job['job_id']}/fields/pressure").content)
    assert max(abs(v) for v in p) == 0.0
    _, speed, _ = _decode(client.get(f"/api/grid-jobs/{job['job_id']}/fields/velocity_magnitude").content)
    assert all(v == pytest.approx(5.0) for v in speed)


def test_projection_settings_are_forwarded(client: TestClient, engine_built) -> None:
    job = client.post(
        "/api/grid-jobs", json=_job(inlet_velocity=8.0, density=1.0, dt=0.01, tolerance=1e-8, max_iterations=5000)
    ).json()
    proj = job["result"]["projection"]
    assert proj["inlet_velocity"] == 8.0 and proj["density"] == 1.0 and proj["dt"] == 0.01
    assert proj["dt_source"] == "explicit"
    assert proj["relative_residual"] <= 1e-8
    assert proj["fluxes"]["inlet"] == pytest.approx(8.0 * 1.2 * 1.2)


def _errors(resp) -> dict[str, str]:
    assert resp.status_code == 422, resp.text
    return {e["field"]: e["message"] for e in resp.json()["errors"]}


@pytest.mark.parametrize(
    ("projection", "field"),
    [
        ({"density": 0}, "projection.density"),
        ({"dt": -1}, "projection.dt"),
        ({"tolerance": 1}, "projection.tolerance"),
        ({"tolerance": 0}, "projection.tolerance"),
        ({"max_iterations": 0}, "projection.max_iterations"),
        ({"max_iterations": 2.5}, "projection.max_iterations"),
        ({"inlet_velocity": -1}, "projection.inlet_velocity"),
        ({"rho": 1.0}, "projection.rho"),
    ],
)
def test_invalid_projection_settings(client: TestClient, projection, field) -> None:
    assert field in _errors(client.post("/api/grid-jobs", json=_job(**projection)))


def test_zero_inflow_requires_dt(client: TestClient) -> None:
    errors = _errors(client.post("/api/grid-jobs", json=_job(inlet_velocity=0)))
    assert errors["projection.dt"] == "must be given explicitly when inlet_velocity is 0"


def test_non_convergence_fails_the_job(client: TestClient, engine_built, settings) -> None:
    body = client.post("/api/grid-jobs", json=_job(max_iterations=2)).json()
    assert body["status"] == "failed"
    assert body["error"]["code"] == "projection_not_converged"
    assert "after 2 iterations" in body["error"]["message"]
    assert body["fields"] == []
    assert not (settings.jobs_dir / body["job_id"]).exists()


def test_inlet_pocket_without_outlet_fails_the_job(client: TestClient, engine_built) -> None:
    job = _job()
    job["test_box"] = {"min": [1.2, 0.0, 0.0], "max": [1.3, 1.2, 1.2]}  # wall across the tunnel
    body = client.post("/api/grid-jobs", json=job).json()
    assert body["status"] == "failed"
    assert body["error"]["code"] == "incompatible_component"
    assert "fed by the inlet" in body["error"]["message"]


def test_diagnostic_field_is_unchanged(client: TestClient, engine_built) -> None:
    job = client.post("/api/grid-jobs", json=_job(fields=["divergence_diagnostic"])).json()
    assert job["result"]["projection"] is None
    meta = job["fields"][0]
    assert meta["provenance"]["kind"] == "diagnostic"
    assert "uniform +x 5 m/s" in meta["provenance"]["description"]
    assert meta["stats"]["max_abs"] == pytest.approx(50.0)


# --- job outcome logging --------------------------------------------------------------------


def test_job_outcomes_are_logged(client: TestClient, engine_built, caplog) -> None:
    import logging

    logger = logging.getLogger("aeroflow.jobs")
    logger.addHandler(caplog.handler)
    try:
        with caplog.at_level(logging.INFO, logger="aeroflow.jobs"):
            done = client.post("/api/grid-jobs", json=_job()).json()
            client.post("/api/grid-jobs", json=_job(fields=[]))
            failed = client.post("/api/grid-jobs", json=_job(max_iterations=2)).json()
            big = _job()
            big["resolution"] = {"nx": 512, "ny": 512, "nz": 512}
            client.post("/api/grid-jobs", json=big)
    finally:
        logger.removeHandler(caplog.handler)
    text = caplog.text
    assert f"job {done['job_id']} completed" in text
    assert "fields_exported=['pressure', 'divergence_before', 'divergence_after', 'velocity_magnitude']" in text
    assert "fields_requested=none" in text and "fields_exported=none" in text
    assert f"job {failed['job_id']} failed" in text and "error=projection_not_converged" in text
    assert "job rejected status=422" in text
