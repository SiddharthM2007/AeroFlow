# AeroFlow

A browser-based 3D virtual wind tunnel. Users import STL/OBJ models, configure
the tunnel and computational grid, and (in later milestones) run a custom C++
incompressible Navier–Stokes solver.

**Current status: Milestone 1, the geometry and computational grid foundation.**
There is no flow solver and no geometry voxelization yet. Solid/fluid
classification uses a procedural axis-aligned test box.

```
AeroFlow/
├── engine/     C++20 grid engine + `aeroflow-grid` CLI (standalone, no web dependencies)
├── backend/    FastAPI service that validates input and invokes the engine executable
├── frontend/   React + TypeScript + Vite + React Three Fiber viewer
└── Makefile    Convenience targets for build, test, and dev servers
```

## Conventions (approved for Milestone 1)

| Topic | Decision |
|---|---|
| Units | SI (meters) |
| Coordinate frame | Flow along **+x**, **y up** (matches Three.js) |
| Domain | `[0, Lx] × [0, Ly] × [0, Lz]`, origin at the domain corner |
| Spacing | `dx = Lx/Nx`, `dy = Ly/Ny`, `dz = Lz/Nz` (cells need not be cubic) |
| Cell centers | `((i+½)dx, (j+½)dy, (k+½)dz)` |
| Linear index | `idx = i + Nx·(j + Ny·k)`, so `i` varies fastest |
| Cell types | `Fluid = 0`, `Solid = 1`, one byte per cell |
| Test-box rule | A cell is **Solid** iff its center lies in the half-open box `[min, max)` |

## Prerequisites

- CMake ≥ 3.20 and a C++20 compiler (tested with Apple Clang 17)
- Python ≥ 3.10
- Node.js ≥ 20 (tested with Node 24) and npm
- Network access on first CMake configure. CMake's FetchContent downloads
  nlohmann/json 3.12 and GoogleTest 1.17.

## Build and run

From the repository root (or use the equivalent `make` targets):

```bash
# 1. C++ engine
cmake -S engine -B engine/build -DCMAKE_BUILD_TYPE=Release
cmake --build engine/build -j

# 2. Backend
python3 -m venv backend/.venv
backend/.venv/bin/pip install -r backend/requirements-dev.txt
cd backend && .venv/bin/uvicorn app.main:app --reload --port 8000     # make dev-backend

# 3. Frontend (in another terminal)
cd frontend && npm install && npm run dev                             # make dev-frontend
```

Open http://localhost:5173. The Vite dev server proxies `/api` to
`http://127.0.0.1:8000`. To use a different backend, set `AEROFLOW_API_URL`.

### Using the viewer

1. **Model:** choose an `.stl` or `.obj` file. It is parsed in the browser,
   recentred on its bounding box, and uploaded to the backend for validation and
   storage.
2. **Transform:** move, rotate, or scale the model with the on-screen gizmo or
   the numeric fields. Use **Fit to tunnel** to scale it to about 30% of the
   smallest tunnel dimension.
3. **Wind tunnel / Grid resolution:** set `Lx, Ly, Lz` and `nx, ny, nz`. The
   lattice preview updates live.
4. **Test solid (box):** set the procedural solid box.
5. **Generate grid:** the backend runs the engine, and the solid cells are
   drawn in orange. Use **Display → Slice** to step through one cell layer at a
   time.

## C++ engine

```bash
engine/build/aeroflow-grid engine/examples/box_in_tunnel.json --no-cells --pretty
echo '{"domain":{"lx":1,"ly":1,"lz":1},"resolution":{"nx":8,"ny":8,"nz":8}}' | engine/build/aeroflow-grid -
```

```
aeroflow-grid <config.json | -> [--output FILE] [--no-cells] [--pretty]
```

Input config (unknown keys are rejected):

```json
{
  "domain":     { "lx": 4.0, "ly": 2.0, "lz": 2.0 },
  "resolution": { "nx": 64,  "ny": 32,  "nz": 32 },
  "test_box":   { "min": [1.0, 0.0, 0.75], "max": [1.5, 0.5, 1.25] }
}
```

`test_box` is optional. The output JSON contains `grid` (resolution, domain,
spacing, cell count/volume, conventions), `classification` (fluid/solid counts
and, unless `--no-cells` is passed, `data`: base64-encoded uint8 per cell in
linear-index order), `test_box`, and `warnings`.

Exit codes: `0` ok, `1` usage, `2` invalid config, `3` I/O or internal error.
On failure, stderr contains `{"error": {"code": ..., "message": ...}}`.

Library layout:

- `Grid3D` (`include/aeroflow/grid3d.hpp`): dimensions, spacing, cell centers,
  `index`/`unravel`, contiguous `std::vector<CellType>` storage, checked
  accessors. The total cell count is capped at 512³.
- `Aabb` and `mark_solid_box` (`box.hpp`): the procedural test solid.
- `parse_config` (`config.hpp`): strict JSON validation with field-path error
  messages.
- `build_grid` and `grid_metadata` (`metadata.hpp`): CLI output.

## Backend API

| Method | Path | Description |
|---|---|---|
| GET | `/api/health` | Service status and engine availability/version |
| POST | `/api/models` | Multipart upload (`file`) of `.stl`/`.obj`; validates structure and returns `model_id` and counts |
| GET | `/api/models/{model_id}` | Uploaded model metadata |
| POST | `/api/grid-jobs` | Validates the grid config, runs the engine, and returns the job with the result |
| GET | `/api/grid-jobs/{job_id}` | Fetch a previous job (kept in memory) |

Grid jobs run synchronously, because the engine finishes in milliseconds at
these sizes. They are stored in memory, and only the last 50 are kept. If the
engine runs but fails, the request still returns `201` with
`status: "failed"` and an `error`.

Validation errors return `422` with
`{"detail": ..., "errors": [{"field": "resolution.nx", "message": ...}]}`. If
the engine binary is missing, the API returns `503`.

Environment variables:

| Variable | Default | Purpose |
|---|---|---|
| `AEROFLOW_ENGINE_PATH` | `engine/build/aeroflow-grid` | Engine executable |
| `AEROFLOW_DATA_DIR` | `backend/data` | Uploaded model storage |
| `AEROFLOW_MAX_CELLS` | `2097152` (128³) | Per-job cell limit (limits the response size) |
| `AEROFLOW_MAX_CELLS_PER_AXIS` | `512` | Per-axis limit |
| `AEROFLOW_MAX_UPLOAD_MB` | `100` | Upload size limit |
| `AEROFLOW_ENGINE_TIMEOUT_S` | `60` | Engine subprocess timeout |

Interactive docs are served at http://127.0.0.1:8000/docs.

## Tests

```bash
make test                     # everything
ctest --test-dir engine/build --output-on-failure        # C++: indexing, coordinates, box classification, config validation
cd backend && .venv/bin/python -m pytest -q              # API validation, uploads, engine round-trip
cd frontend && npm test && npm run build                 # TS unit tests + typecheck + production build
```

The backend tests that run the engine are skipped if `engine/build/aeroflow-grid`
has not been built.

## Not in Milestone 1

These are left out deliberately, and each will need decisions from the
developer:

- Geometry voxelization: classifying cells from the imported mesh, including
  the inside test and the treatment of partially filled cells
- The Navier–Stokes solver: discretization, variable arrangement, time
  integration, pressure solve
- Boundary conditions: inlet, outlet, walls, and solid surfaces
- Non-uniform or refined grids
