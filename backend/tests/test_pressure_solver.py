"""Milestone 7: pressure solver selection, diagnostics, fallback reporting and preflight estimates."""

from __future__ import annotations

import dataclasses
import logging
import time
from array import array

import pytest
from fastapi.testclient import TestClient

from app.engine import GridEngine
from app.fields import parse_field_header
from app.main import create_app
from app.schemas import GridJobRequest
from app.simulation_estimate import AUTO_MULTIGRID_MIN_CELLS, estimate_simulation, selected_solver

PROJECTION_FIELDS = ["pressure", "divergence_after", "velocity_magnitude"]


def _projection_job(solver: str | None = None) -> dict:
    job = {
        "domain": {"lx": 2.4, "ly": 1.2, "lz": 1.2},
        "resolution": {"nx": 24, "ny": 12, "nz": 12},  # 3,456 cells: auto selects MG-PCG
        "test_box": {"min": [0.8, 0.3, 0.3], "max": [1.2, 0.9, 0.9]},
        "fields": PROJECTION_FIELDS,
        "include_cells": False,
        "projection": {"tolerance": 1e-10},
    }
    if solver is not None:
        job["projection"]["pressure_solver"] = solver
    return job


def _simulation_job(solver: str | None = None, resolution=(24, 12, 12), **simulation) -> dict:
    sim = {"duration": 0.04, "frame_interval": 0.02, **simulation}
    if solver is not None:
        sim["pressure_solver"] = solver
    nx, ny, nz = resolution
    return {
        "domain": {"lx": 2.4, "ly": 1.2, "lz": 1.2},
        "resolution": {"nx": nx, "ny": ny, "nz": nz},
        "test_box": {"min": [0.8, 0.3, 0.3], "max": [1.2, 0.9, 0.9]},
        "fields": ["sim_pressure", "sim_velocity_magnitude"],
        "include_cells": False,
        "simulation": sim,
    }


def _wait(client: TestClient, job_id: str, timeout: float = 60.0) -> dict:
    deadline = time.time() + timeout
    while time.time() < deadline:
        job = client.get(f"/api/grid-jobs/{job_id}").json()
        if job["status"] in ("completed", "stopped", "failed", "cancelled"):
            return job
        time.sleep(0.05)
    raise AssertionError(f"job {job_id} did not finish in {timeout}s")


def _values(client: TestClient, job_id: str, name: str, frame: int = 0) -> array:
    data = client.get(f"/api/grid-jobs/{job_id}/fields/{name}?frame={frame}").content
    header = parse_field_header(data)
    block = next(b for b in header["blocks"] if b["name"] == "values")
    values = array("f")
    values.frombytes(data[block["offset"] : block["offset"] + block["length"]])
    return values


# --- validation and configuration ---------------------------------------------------------------


@pytest.mark.parametrize("block", ["projection", "simulation"])
@pytest.mark.parametrize("bad", ["multigrid", "MGPCG", 1, None])
def test_invalid_solver_is_rejected(client: TestClient, block: str, bad) -> None:
    job = _projection_job() if block == "projection" else _simulation_job()
    job[block]["pressure_solver"] = bad
    resp = client.post("/api/grid-jobs", json=job)
    assert resp.status_code == 422, resp.text
    assert f"{block}.pressure_solver" in {e["field"] for e in resp.json()["errors"]}


def test_solver_defaults_to_auto_and_reaches_the_engine_config() -> None:
    req = GridJobRequest.model_validate(_projection_job())
    assert req.projection.pressure_solver == "auto"
    assert req.engine_config(fields_dir="/tmp/x")["projection"]["pressure_solver"] == "auto"
    sim = GridJobRequest.model_validate(_simulation_job("jacobi_pcg"))
    assert sim.engine_config(fields_dir="/tmp/x")["simulation"]["pressure_solver"] == "jacobi_pcg"


# --- preflight estimates ------------------------------------------------------------------------


def test_auto_estimates_with_the_solver_it_selects() -> None:
    assert selected_solver("auto", AUTO_MULTIGRID_MIN_CELLS - 1) == "jacobi_pcg"
    assert selected_solver("auto", AUTO_MULTIGRID_MIN_CELLS) == "mgpcg"
    assert selected_solver("jacobi_pcg", 10**7) == "jacobi_pcg"

    small = estimate_simulation(GridJobRequest.model_validate(_simulation_job(resolution=(12, 6, 6))))  # 432 cells
    assert small["solver_requested"] == "auto" and small["solver"] == "jacobi_pcg"
    assert small["multigrid_bytes"] == 0

    big = {"resolution": (128, 64, 64), "duration": 0.02, "frame_interval": 0.01}
    res = big.pop("resolution")
    auto = estimate_simulation(GridJobRequest.model_validate(_simulation_job(resolution=res, **big)))
    mg = estimate_simulation(GridJobRequest.model_validate(_simulation_job("mgpcg", resolution=res, **big)))
    jac = estimate_simulation(GridJobRequest.model_validate(_simulation_job("jacobi_pcg", resolution=res, **big)))
    assert auto["solver"] == "mgpcg" and auto["seconds"] == mg["seconds"]
    assert jac["solver"] == "jacobi_pcg"
    assert auto["steps"] == jac["steps"]  # the time step does not depend on the solver
    # Calibrated per-step pressure cost at 524k cells (measured: MG-PCG 0.13-0.20 s, Jacobi-PCG 2.0-2.2 s).
    assert 0.13 <= mg["pressure_step_seconds"] <= 0.35
    assert 2.0 <= jac["pressure_step_seconds"] <= 3.5
    assert jac["seconds"] > 5 * mg["seconds"]
    assert mg["alternative_solver"] == "jacobi_pcg" and mg["alternative_seconds"] == pytest.approx(jac["seconds"])
    assert mg["multigrid_bytes"] > 0 and mg["memory_bytes"] > jac["memory_bytes"]


def test_mg_runs_are_not_rejected_by_the_jacobi_model(settings) -> None:
    # A budget that the Jacobi-PCG model exceeds but MG-PCG fits comfortably.
    client = TestClient(create_app(dataclasses.replace(settings, sim_timeout_s=30.0)))
    job = _simulation_job(resolution=(128, 64, 64), duration=0.02, frame_interval=0.01)
    job["domain"] = {"lx": 4.0, "ly": 2.0, "lz": 2.0}
    auto = client.post("/api/grid-jobs/estimate", json=job).json()
    assert auto["ok"] is True, auto["problems"]
    assert auto["estimate"]["solver"] == "mgpcg"
    job["simulation"]["pressure_solver"] = "jacobi_pcg"
    jac = client.post("/api/grid-jobs/estimate", json=job).json()
    assert jac["ok"] is False
    message = next(p for p in jac["problems"] if "budget" in p)
    assert "Jacobi-PCG" in message and "or the MG-PCG pressure solver" in message


# --- jobs through the engine --------------------------------------------------------------------


def test_projection_jobs_report_the_solver_and_agree(client: TestClient, engine_built) -> None:
    results = {}
    for solver in ("jacobi_pcg", "mgpcg", "auto"):
        job = client.post("/api/grid-jobs", json=_projection_job(solver)).json()
        assert job["status"] == "completed", job["error"]
        diag = job["result"]["projection"]["pressure_solver"]
        assert diag["requested"] == solver
        assert diag["selected"] == ("jacobi_pcg" if solver == "jacobi_pcg" else "mgpcg")
        assert diag["used"] == diag["selected"]
        assert diag["fallback"] is False and diag["fallback_reason"] is None
        assert (diag["multigrid_levels"] >= 2) == (diag["used"] == "mgpcg")
        assert diag["pcg_ms"] >= 0
        results[solver] = job
    it = {s: j["result"]["projection"]["iterations"] for s, j in results.items()}
    assert it["mgpcg"] < it["jacobi_pcg"] / 5
    # Same fields from both solvers (identical AFFIELD layout; values agree to the solver tolerance).
    for name in PROJECTION_FIELDS:
        a = _values(client, results["jacobi_pcg"]["job_id"], name)
        b = _values(client, results["mgpcg"]["job_id"], name)
        assert len(a) == len(b)
        if name == "divergence_after":
            # Residual-level noise for both solvers: compare with the physical scale U / h = 50 1/s.
            assert max(abs(v) for v in a) <= 1e-6 * 50 and max(abs(v) for v in b) <= 1e-6 * 50
            continue
        scale = max(abs(v) for v in a)
        assert max(abs(x - y) for x, y in zip(a, b)) <= 1e-6 * scale, name


def test_simulation_jobs_report_solver_diagnostics(client: TestClient, engine_built) -> None:
    runs = {}
    for solver in ("jacobi_pcg", "mgpcg"):
        started = client.post("/api/grid-jobs", json=_simulation_job(solver)).json()
        assert started["estimate"]["solver"] == solver
        job = _wait(client, started["job_id"])
        assert job["status"] == "completed", job["error"]
        sim = job["result"]["simulation"]
        diag = sim["pressure_solver"]
        assert diag["requested"] == solver and diag["selected"] == solver and diag["used"] == solver
        assert diag["fallbacks"] == 0 and diag["fallback_reason"] is None
        assert diag["solve_seconds"] > 0 and diag["pressure_iterations"] > 0
        assert set(sim["history"]["solver"]) == {solver}
        assert len(sim["history"]["solve_ms"]) == sim["steps"] + 1
        assert sim["settings"]["pressure_solver"] == solver
        for f in job["fields"]:  # every frame records the solver that produced it
            assert f["provenance"]["pressure_solver"] == solver
            assert "Jacobi-PCG or MG-PCG" in f["provenance"]["description"]
        if solver == "mgpcg":
            assert diag["multigrid_levels"] >= 2 and diag["multigrid_setups"] == 1
            assert diag["multigrid_setup_ms"] > 0 and diag["multigrid_memory_bytes"] > 0
        else:
            assert diag["multigrid_levels"] == 0 and diag["multigrid_memory_bytes"] == 0
        runs[solver] = job
    # Same frames and field layout; the final pressure agrees to the solver tolerance.
    names = lambda j: sorted((f["name"], f["frame"]["index"]) for f in j["fields"])  # noqa: E731
    assert names(runs["jacobi_pcg"]) == names(runs["mgpcg"])
    a = _values(client, runs["jacobi_pcg"]["job_id"], "sim_pressure", 2)
    b = _values(client, runs["mgpcg"]["job_id"], "sim_pressure", 2)
    assert max(abs(x - y) for x, y in zip(a, b)) <= 1e-4 * max(abs(v) for v in a)


def test_fallback_is_reported_and_logged(settings, engine_built, monkeypatch, caplog) -> None:
    # The engine reports a multigrid fallback; the backend must pass it through and log a warning.
    fake = {
        "simulation": {
            "status": "completed",
            "steps": 3,
            "time": 0.04,
            "frames": 1,
            "pressure_solver": {
                "requested": "mgpcg",
                "selected": "mgpcg",
                "used": "jacobi_pcg",
                "fallbacks": 2,
                "fallback_reason": "multigrid preconditioner lost positivity (r.z = nan) at iteration 4",
                "multigrid_levels": 3,
            },
        },
        "fields": [],
    }
    monkeypatch.setattr(GridEngine, "run_streaming", lambda self, *a, **k: fake)
    client = TestClient(create_app(settings))
    logger = logging.getLogger("aeroflow.jobs")
    logger.addHandler(caplog.handler)
    try:
        with caplog.at_level(logging.INFO, logger="aeroflow.jobs"):
            job = _wait(client, client.post("/api/grid-jobs", json=_simulation_job("mgpcg")).json()["job_id"])
    finally:
        logger.removeHandler(caplog.handler)
    diag = job["result"]["simulation"]["pressure_solver"]
    assert diag["fallbacks"] == 2 and "lost positivity" in diag["fallback_reason"]
    line = next(r for r in caplog.records if job["job_id"] in r.getMessage() and "completed" in r.getMessage())
    assert line.levelno == logging.WARNING
    assert "FALLBACK" in line.getMessage() and "solver=jacobi_pcg (requested mgpcg)" in line.getMessage()
