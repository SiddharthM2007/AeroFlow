"""Milestone 6: asynchronous Navier-Stokes jobs, preflight, progress, frames and cancellation."""

from __future__ import annotations

import dataclasses
import time
from array import array
from datetime import datetime, timezone

import pytest
from fastapi.testclient import TestClient

import app.main as main_module
from app.fields import parse_field_header
from app.jobs import JobStore
from app.main import create_app
from app.schemas import GridJob, GridJobRequest


def _job(fields=("sim_pressure", "sim_velocity_magnitude", "sim_divergence"), **simulation) -> dict:
    sim = {"duration": 0.04, "frame_interval": 0.02}
    sim.update(simulation)
    return {
        "domain": {"lx": 1.6, "ly": 0.6, "lz": 0.6},
        "resolution": {"nx": 16, "ny": 6, "nz": 6},  # h = 0.1
        "test_box": {"min": [0.4, 0.2, 0.2], "max": [0.6, 0.4, 0.4]},
        "fields": list(fields),
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


def _decode(data: bytes) -> tuple[dict, array]:
    header = parse_field_header(data)
    block = next(b for b in header["blocks"] if b["name"] == "values")
    values = array("f")
    values.frombytes(data[block["offset"] : block["offset"] + block["length"]])
    return header, values


def test_simulation_runs_asynchronously_with_frames(client: TestClient, engine_built) -> None:
    resp = client.post("/api/grid-jobs", json=_job())
    assert resp.status_code == 201, resp.text
    started = resp.json()
    assert started["status"] in ("queued", "running")
    assert started["estimate"]["frames"] == 3
    assert started["estimate"]["steps"] > 0

    job = _wait(client, started["job_id"])
    assert job["status"] == "completed", job["error"]
    sim = job["result"]["simulation"]
    assert sim["status"] == "completed"
    assert sim["frames"] == 3 and sim["frame_times"] == pytest.approx([0.0, 0.02, 0.04])
    assert job["progress"]["time"] == pytest.approx(0.04)
    hist = sim["history"]
    assert all(leak == 0.0 for leak in hist["wall_leak"])
    for q_in, q_out in zip(hist["inflow"], hist["outflow"]):
        assert q_out == pytest.approx(q_in, rel=1e-6)

    names = {(f["name"], f["frame"]["index"]) for f in job["fields"]}
    assert names == {(n, i) for n in ("sim_pressure", "sim_velocity_magnitude", "sim_divergence") for i in range(3)}
    for f in job["fields"]:
        assert f["provenance"]["kind"] == "simulation" and f["provenance"]["solved"] is True

    for frame, t in enumerate([0.0, 0.02, 0.04]):
        r = client.get(f"/api/grid-jobs/{job['job_id']}/fields/sim_pressure?frame={frame}")
        assert r.status_code == 200
        header, _ = _decode(r.content)
        assert header["frame"]["index"] == frame and header["frame"]["time_s"] == pytest.approx(t)
    assert client.get(f"/api/grid-jobs/{job['job_id']}/fields/sim_pressure?frame=3").status_code == 404


def test_velocity_components_use_face_locations(client: TestClient, engine_built) -> None:
    job = _wait(client, client.post("/api/grid-jobs", json=_job(fields=["sim_u", "sim_w"])).json()["job_id"])
    assert job["status"] == "completed", job["error"]
    header, values = _decode(client.get(f"/api/grid-jobs/{job['job_id']}/fields/sim_u?frame=0").content)
    assert header["location"] == "face_x" and header["shape"] == [17, 6, 6]
    assert values[0] == pytest.approx(5.0)  # inlet face keeps U_in
    header, _ = _decode(client.get(f"/api/grid-jobs/{job['job_id']}/fields/sim_w?frame=2").content)
    assert header["location"] == "face_z" and header["shape"] == [16, 6, 7]


def test_estimate_endpoint(client: TestClient) -> None:
    body = client.post("/api/grid-jobs/estimate", json=_job()).json()
    assert body["ok"] is True
    est = body["estimate"]
    assert est["cells"] == 576 and est["frames"] == 3
    assert est["dt"] == pytest.approx(0.5 / (1.6 * 5.0 * 30 + 2 * 1.5e-5 * 300))
    assert est["output_bytes"] > 0 and est["seconds"] > 0
    big = _job(duration=30.0, frame_interval=1.0)
    big["resolution"] = {"nx": 128, "ny": 64, "nz": 64}
    body = client.post("/api/grid-jobs/estimate", json=big).json()
    assert body["ok"] is False
    assert any("max_steps" in p for p in body["problems"])
    assert any("budget" in p for p in body["problems"])
    no_sim = _job(fields=["pressure"])
    assert client.post("/api/grid-jobs/estimate", json=no_sim).status_code == 400


def test_preflight_rejects_runs_over_budget(client: TestClient) -> None:
    big = _job(duration=30.0, frame_interval=1.0)
    big["resolution"] = {"nx": 128, "ny": 64, "nz": 64}
    resp = client.post("/api/grid-jobs", json=big)
    assert resp.status_code == 422
    assert "rejected before starting" in resp.json()["detail"]


@pytest.mark.parametrize(
    ("sim_settings", "field"),
    [
        ({"cfl": 0}, "simulation.cfl"),
        ({"cfl": 1.5}, "simulation.cfl"),
        ({"viscosity": -1}, "simulation.viscosity"),
        ({"duration": 0}, "simulation.duration"),
        ({"frame_interval": -0.1}, "simulation.frame_interval"),
        ({"max_steps": 0}, "simulation.max_steps"),
        ({"initial": "random"}, "simulation.initial"),
        ({"duration": 100, "frame_interval": 0.001}, "simulation.frame_interval"),
        ({"nu": 1e-5}, "simulation.nu"),
    ],
)
def test_invalid_simulation_settings(client: TestClient, sim_settings, field) -> None:
    resp = client.post("/api/grid-jobs", json=_job(**sim_settings))
    assert resp.status_code == 422, resp.text
    assert field in {e["field"] for e in resp.json()["errors"]}


def test_unstable_user_dt_fails_the_job(client: TestClient, engine_built, settings) -> None:
    job = _wait(client, client.post("/api/grid-jobs", json=_job(dt=0.5)).json()["job_id"])
    assert job["status"] == "failed"
    assert job["error"]["code"] == "unstable_timestep"
    assert not (settings.jobs_dir / job["job_id"]).exists()


def test_cancel_stops_a_running_simulation(client: TestClient, engine_built, settings) -> None:
    long = _job(duration=5.0, frame_interval=1.0, max_steps=20_000)  # ~7,200 steps, ~2 min estimate
    long["resolution"] = {"nx": 48, "ny": 18, "nz": 18}
    started = client.post("/api/grid-jobs", json=long).json()
    deadline = time.time() + 30
    while client.get(f"/api/grid-jobs/{started['job_id']}").json()["status"] != "running" and time.time() < deadline:
        time.sleep(0.05)
    resp = client.post(f"/api/grid-jobs/{started['job_id']}/cancel")
    assert resp.status_code == 200
    job = _wait(client, started["job_id"])
    assert job["status"] == "cancelled"
    assert job["fields"] == []
    assert not (settings.jobs_dir / started["job_id"]).exists()
    assert client.post(f"/api/grid-jobs/{started['job_id']}/cancel").status_code == 409


def test_wall_time_budget_stops_and_keeps_frames(settings, engine_built, monkeypatch) -> None:
    # Make the preflight optimistic so the run starts, then let the engine's hard stop act.
    real = main_module.estimate_simulation
    monkeypatch.setattr(main_module, "estimate_simulation", lambda req: {**real(req), "seconds": 0.0})
    client = TestClient(create_app(dataclasses.replace(settings, sim_timeout_s=0.3)))
    long = _job(duration=5.0, frame_interval=0.01, max_steps=20_000)
    long["resolution"] = {"nx": 48, "ny": 18, "nz": 18}
    job = _wait(client, client.post("/api/grid-jobs", json=long).json()["job_id"])
    assert job["status"] == "stopped"
    assert job["error"]["code"] == "wall_time_budget"
    sim = job["result"]["simulation"]
    assert sim["status"] == "stopped" and sim["time"] < 5.0
    assert len(job["fields"]) >= 3  # frame 0 at least, for each field


def test_running_jobs_are_not_evicted() -> None:
    store = JobStore(max_jobs=1)
    req = GridJobRequest.model_validate(_job())
    now = datetime.now(timezone.utc)
    store.add(GridJob(job_id="running", status="running", created_at=now, request=req))
    store.add(GridJob(job_id="done", status="completed", created_at=now, request=req))
    store.add(GridJob(job_id="new", status="completed", created_at=now, request=req))
    assert store.get("running") is not None
    assert store.get("done") is None
