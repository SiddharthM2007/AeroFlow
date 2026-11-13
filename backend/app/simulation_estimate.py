"""Preflight cost estimate for Navier-Stokes jobs (Milestones 6-7).

The pressure solve dominates every step, and its cost depends on the solver, so each solver has its
own model, calibrated on the Milestone 7 phase 4 benchmarks (Apple M2 Pro, single thread, box and
Valve Body STL obstacles, tolerance 1e-8, 65k-2.1M cells):

  Jacobi-PCG: iterations per step ~ 6.7-7.4 N^(1/3) (271 at 65k, 545-595 at 524k, 838-873 at 2.1M)
              at ~6.4-7.2 ns per cell and iteration       -> model 7.0 N^(1/3) x 7.5 ns
  MG-PCG:     6.6-12 iterations per step at every size (max 14), ~33-35 ns per cell and iteration
              (one V-cycle plus the PCG work)              -> model 12 x 36 ns
  Other work per step (momentum, boundary conditions, divergence, statistics): 113-120 ns per cell
              -> model 120 ns

The models are deliberately on the high side (MG-PCG: 0.9 s per step modelled vs 0.66-0.87 s
measured at 2.1M cells). "auto" is estimated with the solver the engine will actually select. The
engine still enforces the wall-time budget as a hard stop.
"""

from __future__ import annotations

import math
from typing import Any

from .schemas import FACE_FIELDS, SIMULATION_FIELDS, GridJobRequest

SOLVER_NAMES = {"mgpcg": "MG-PCG", "jacobi_pcg": "Jacobi-PCG", "auto": "Auto"}

# Mirrors aeroflow::kAutoMultigridMinUnknowns: auto uses MG-PCG from this many fluid cells.
AUTO_MULTIGRID_MIN_CELLS = 1000

OTHER_SECONDS_PER_CELL_STEP = 1.2e-7
JACOBI_ITERATIONS_PER_CUBE_ROOT_CELL = 7.0
JACOBI_SECONDS_PER_CELL_ITERATION = 7.5e-9
JACOBI_INITIAL_FACTOR = 1.25  # the cold initial projection needs ~1.2x a warm-started step
MG_ITERATIONS_PER_STEP = 12.0
MG_INITIAL_ITERATIONS = 16.0
MG_SECONDS_PER_CELL_ITERATION = 3.6e-8
MG_SETUP_SECONDS_PER_CELL = 5e-8
PEAK_SPEED_FACTOR = 1.6
BYTES_PER_CELL = 300  # simulation state (measured ~155 B/cell, kept conservative)
MG_BYTES_PER_CELL = 30  # multigrid hierarchy (measured 22-36 B/cell, anisotropic grids highest)
HEADER_BYTES = 64 * 1024


def selected_solver(requested: str, fluid_cells: int) -> str:
    """The solver the engine uses for this request ("auto" resolved by grid size)."""
    if requested != "auto":
        return requested
    return "mgpcg" if fluid_cells >= AUTO_MULTIGRID_MIN_CELLS else "jacobi_pcg"


def pressure_model(solver: str, cells: int) -> dict[str, float]:
    """Seconds and iterations per step, initial projection and setup for one solver."""
    if solver == "mgpcg":
        per_iteration = cells * MG_SECONDS_PER_CELL_ITERATION
        return {
            "iterations_per_step": MG_ITERATIONS_PER_STEP,
            "pressure_step_seconds": MG_ITERATIONS_PER_STEP * per_iteration,
            "initial_seconds": MG_INITIAL_ITERATIONS * per_iteration,
            "setup_seconds": cells * MG_SETUP_SECONDS_PER_CELL,
        }
    iterations = JACOBI_ITERATIONS_PER_CUBE_ROOT_CELL * cells ** (1 / 3)
    per_iteration = cells * JACOBI_SECONDS_PER_CELL_ITERATION
    return {
        "iterations_per_step": iterations,
        "pressure_step_seconds": iterations * per_iteration,
        "initial_seconds": JACOBI_INITIAL_FACTOR * iterations * per_iteration,
        "setup_seconds": 0.0,
    }


def estimate_simulation(req: GridJobRequest) -> dict[str, Any]:
    s = req.simulation_settings
    r, d = req.resolution, req.domain
    cells = r.total  # fluid cells are not known before voxelization; all cells is the upper bound
    h = (d.lx / r.nx, d.ly / r.ny, d.lz / r.nz)
    rate = PEAK_SPEED_FACTOR * s.inlet_velocity * sum(1 / x for x in h) + 2 * s.viscosity * sum(1 / x**2 for x in h)
    dt = s.cfl / rate if rate > 0 else s.frame_interval
    if s.dt is not None:
        dt = min(dt, s.dt)
    dt = min(dt, s.frame_interval)
    steps = int(math.ceil(s.duration / dt)) + s.frame_count  # frame landings shorten some steps

    solver = selected_solver(s.pressure_solver, cells)
    # With no inflow the right-hand side is zero and the pressure solve is free.
    model = pressure_model(solver, cells if s.inlet_velocity > 0 else 0)
    other = cells * OTHER_SECONDS_PER_CELL_STEP
    step_seconds = model["pressure_step_seconds"] + other
    # The other solver, for comparison in the UI and in rejection messages.
    alternative = "jacobi_pcg" if solver == "mgpcg" else "mgpcg"
    alt = pressure_model(alternative, cells)
    alt_seconds = alt["setup_seconds"] + alt["initial_seconds"] + steps * (alt["pressure_step_seconds"] + other)

    face_counts = {
        "sim_u": (r.nx + 1) * r.ny * r.nz,
        "sim_v": r.nx * (r.ny + 1) * r.nz,
        "sim_w": r.nx * r.ny * (r.nz + 1),
    }
    requested = [f for f in req.fields if f in SIMULATION_FIELDS]
    per_frame = {f: (face_counts[f] if f in FACE_FIELDS else cells) * 5 + HEADER_BYTES for f in requested}
    multigrid_bytes = cells * MG_BYTES_PER_CELL if solver == "mgpcg" else 0
    return {
        "cells": cells,
        "spacing": list(h),
        "dt": dt,
        "steps": steps,
        "frames": s.frame_count,
        "solver_requested": s.pressure_solver,
        "solver": solver,
        "iterations_per_step": round(model["iterations_per_step"]),
        "step_seconds": step_seconds,
        "pressure_step_seconds": model["pressure_step_seconds"],
        "setup_seconds": model["setup_seconds"],
        "seconds": model["setup_seconds"] + model["initial_seconds"] + steps * step_seconds,
        "alternative_solver": alternative,
        "alternative_seconds": alt_seconds,
        "output_bytes": s.frame_count * sum(per_frame.values()),
        "largest_file_bytes": max(per_frame.values(), default=0),
        "memory_bytes": cells * BYTES_PER_CELL + multigrid_bytes,
        "multigrid_bytes": multigrid_bytes,
    }
