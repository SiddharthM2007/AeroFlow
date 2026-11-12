# AeroFlow

A browser-based 3D virtual wind tunnel. Users import STL/OBJ models, configure
the tunnel and computational grid, and (in later milestones) run a custom C++
incompressible Navier–Stokes solver.

**Current status: Milestone 4A, scientific field visualization.** It builds on
Milestone 1 (the grid foundation), 2A (geometry-aware tunnel fitting), 2B (CAD
mesh voxelization) and 3A (staggered MAC grid and discrete divergence).

- **Field pipeline:** numerical fields now travel from C++ through FastAPI to the
  browser in a versioned binary format (AFFIELD).
- **Viewer:** fields are shown as colored 3D slices with a legend.
- **Boundary conditions (3B):** the C++ MAC grid has reusable boundary-condition
  operators for the future solver.
- **First field:** a *divergence diagnostic* of an initialized velocity field.
  There is no advection, diffusion, pressure solve or time stepping yet, so
  nothing shown is a solved airflow.

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

## Model placement (approved for Milestone 2A)

Imported vertex positions are never modified. A model is placed by:

```
p_world = T(position) · R(rotation) · S(scale) · U(units) · (p_raw − pivot)
```

| Term | Meaning |
|---|---|
| `p_raw` | Vertex as stored in the file, in file units |
| `pivot` | Center of the raw bounding box, in file units. Rotation and scale happen about it |
| `U` | Unit conversion to meters: mm 0.001, cm 0.01, m 1, in 0.0254. Always derived from the selected unit, so it never stacks |
| `S` | User scale per axis, treated as an intentional physical change |
| `R` | Euler XYZ in degrees, with matrix `R = Rx·Ry·Rz` (the Three.js `"XYZ"` order) |
| `T` | Position of the pivot in tunnel coordinates, in meters |

Grid jobs can carry a `model_transform` with these parameters plus the composed
4×4 row-major `matrix` (raw file units → tunnel meters). The backend checks that
the matrix matches the parameters. `backend/app/transform.py` is the reference
implementation that the Milestone 2B voxelizer has to match.

**Fit Tunnel to Model** works as follows:

1. Take the model's exact world-space bounding box, computed from every vertex
   after the full transform.
2. Set clearances as multiples of `D = max(bx, by, bz)`. Defaults: upstream 2D,
   downstream 5D, 1.5D on each side (z), 1.5D on top, and 1.5D below. The model
   floats freely and does not touch the floor.
3. Keep the current cell spacing `d` exactly: `n = ceil(L_required / d)` and
   `L = n·d`. The leftover fraction of a cell is split evenly between the two
   sides of each axis.
4. Move the model to its new place by changing only `position`. Units, scale and
   rotation are never changed.
5. If the result is over the server grid budget (`GET /api/limits`), offer a
   coarser grid, made by scaling all spacings by the same factor until both the
   total-cell and per-axis limits are met. The alternative is to apply the
   tunnel at the current spacing and adjust it by hand.
6. Reframe the camera on the tunnel and the model.

## Mesh voxelization (approved for Milestone 2B)

**Cell rule.** Cell `(i,j,k)` is **Solid** if and only if its center
`((i+½)Δx, (j+½)Δy, (k+½)Δz)` is inside the closed mesh. Classification is binary.
A cell is never marked solid just because the surface passes through part of it.
The mask format is unchanged: one byte per cell, `idx = i + Nx·(j + Ny·k)`.

**Data flow.**

```
stored STL/OBJ (file units) --backend: parse (float32, like Three.js), apply validated matrix-->
  world-space triangles, meters (binary .aftri file) --engine: validate -> BVH -> ray parity--> Grid3D mask
```

**Backend → engine geometry.** The engine receives triangles already transformed
into tunnel coordinates in meters, so it applies no further transformation.

- **File format:** `"AFTRI1\0\0"`, then a little-endian `uint64` triangle count,
  then `n × 9 float64` values.
- **Engine config:** the file is passed as `{"mesh": {"triangles_file": ...}}`.
- **Temporary file:** it is named by the job id (never by the client's filename)
  and deleted after the job.
- **Matching the viewer:** the backend parses the raw file the same way the viewer
  does. Coordinates are rounded to float32, OBJ polygons are fan-triangulated, and
  negative indices are resolved like Three.js. It applies the same row-major matrix
  in the same order as the frontend. A shared fixture
  (`fixtures/geometry/box_quads.obj`) checks that both sides produce identical
  world bounds.

**Algorithm** (`engine/src/voxelize.cpp`):

- **One ray per grid row:** a single +x ray is cast along each `(j,k)` row. The
  parity for cell `i` is the number of crossings with `x > x_i`, which is the same
  as casting one ray per cell. Rows and cells outside the mesh bounds are set to
  Fluid without any tests.
- **BVH:** splits at the median triangle centroid along the longest axis. Leaf
  size defaults to 4 (`mesh.bvh_leaf_size`). Traversal is iterative, and the
  slab ray–box test handles zero direction components.
- **Ray–triangle test:** Möller–Trumbore with three outcomes: `Hit`, `Miss` or
  `Ambiguous`. A crossing within a barycentric tolerance (1e-9) of an edge or
  vertex is Ambiguous, and so is a ray lying in a triangle's plane. Shared edges
  and vertices therefore fall in the Ambiguous band for *every* adjacent triangle
  and are never counted twice.
- **Fallback:** a cell whose parity depends on an ambiguous crossing is recast
  from its center along six fixed, non-axis-aligned directions. The first ray
  with no ambiguous hits decides. This is deterministic, with no random jitter.
- **Surface policy:** a center within `1e-10 × max(domain diagonal, mesh
  diagonal)` of the surface is **Solid**, and so is the (not observed in tests)
  case where every fallback direction is ambiguous.

**Validation** (`engine/src/mesh.cpp`). Vertices are welded by **exact**
coordinate equality.

| Check | Result |
|---|---|
| Empty mesh, non-finite coordinates | error |
| Duplicate triangles | error |
| Non-manifold edges (shared by more than 2 triangles) | error |
| Open boundary edges (not watertight) | error, including gaps of 1e-9 m, since there is no tolerance welding |
| Degenerate (zero-area) triangles | removed, with a warning |
| Inconsistent winding | warning; classification is unaffected, but no volume is reported |
| Mesh partly outside the tunnel | warning; the part inside the domain is classified |

**Volume check.** The mesh volume is `V = ⅙ Σ v0·(v1×v2)`. When the mesh is fully
inside the domain, the result reports `ε_V = |N_solid·ΔxΔyΔz − V| / V`.

**Measured accuracy.** Sphere, r = 0.3 m, 9,024 triangles, unit domain:

| N³ | V error |
|---|---|
| 16³ | 3.80 % |
| 32³ | 1.39 % |
| 64³ | 0.44 % |
| 128³ | 0.047 % |

**Measured performance.** Full request on Apple Silicon: upload, Python parse
and transform, engine run.

| Triangles | Grid | Request | BVH build | Classification |
|---|---|---|---|---|
| 65k | 192×128² | 0.11 s | 13 ms | 4 ms |
| 261k | 192×128² | 0.44 s | 60 ms | 6 ms |
| 638k | 192×128² | 1.16 s | 157 ms | 8 ms |

Pure-Python parsing and transforming dominates the request time, at about 0.5 µs
per vertex. On a 9k-triangle sphere at 48³, the BVH runs **648× fewer**
ray–triangle tests than brute force.

**Known limitations.**

- **Self-intersecting meshes, overlapping shells and multi-body assemblies that
  interpenetrate:** parity counting gives XOR-like results (overlapping regions
  become fluid). These cases are *not* detected; repair or union the shells first.
- **Thin features below the cell size:** they disappear, because only cell centers
  are sampled. The UI warns when the model's thinnest dimension spans fewer than 4
  cells.
- **Open CAD exports with tiny gaps:** these are rejected rather than repaired.
- **Triangle limit:** 1,000,000 by default (`AEROFLOW_MAX_TRIANGLES`). Python
  parsing of very large meshes takes seconds.
- **Engine run time:** limited by the existing `AEROFLOW_ENGINE_TIMEOUT_S`.

## Staggered MAC grid (approved for Milestone 3A)

The engine code is in `engine/include/aeroflow/mac_grid.hpp` and
`divergence.hpp`. It is not used by the CLI, backend or frontend yet.

**Field placement.** `MacGrid` covers the same cells as `Grid3D`, built with
`MacGrid(grid)`. The spacing `dx = Lx/Nx` is computed with the same expression,
so pressure positions equal `Grid3D::cell_center` bit for bit.

| Field | Extents | Location (m) |
|---|---|---|
| `p(i,j,k)` | `Nx × Ny × Nz` | `((i+½)dx, (j+½)dy, (k+½)dz)`, cell centers |
| `u(i,j,k)` | `(Nx+1) × Ny × Nz` | `(i·dx, (j+½)dy, (k+½)dz)`, x-faces |
| `v(i,j,k)` | `Nx × (Ny+1) × Nz` | `((i+½)dx, j·dy, (k+½)dz)`, y-faces |
| `w(i,j,k)` | `Nx × Ny × (Nz+1)` | `((i+½)dx, (j+½)dy, k·dz)`, z-faces |

**Storage.**

- Each field is a separate contiguous `std::vector<double>` (`StaggeredArray`),
  indexed x-fastest with its own extents: `idx = i + n0·(j + n1·k)`.
- Cell `(i,j,k)` is bounded by `u(i)`/`u(i+1)`, `v(j)`/`v(j+1)` and `w(k)`/`w(k+1)`.
- `at()` is bounds-checked; `operator()` is not.

**Allocation guard.** Sizes are computed with overflow checks. Grids larger than
`Grid3D`'s 512³ cells are rejected. The four fields together may not exceed a
byte limit, 2 GiB by default and configurable per grid. At about 32 bytes per
cell, 256³ fits and 512³ is rejected before anything is allocated.

**Face classification.** `classify_faces(mask)` returns arrays shaped exactly
like `u`, `v` and `w`.

| Cells on either side | `FaceType` | Active |
|---|---|---|
| fluid \| fluid | `Fluid` | yes |
| fluid \| solid | `SolidBoundary` (obstacle wall) | no |
| solid \| solid | `SolidInterior` | no |
| tunnel wall \| fluid | `DomainBoundary` (tunnel wall) | yes |
| tunnel wall \| solid | `SolidInterior` (tunnel wall covered by the obstacle) | no |

Obstacle walls and tunnel walls are never conflated. No physical boundary
conditions are applied yet.

**Initialization.**

- `set_zero()`: zero velocity and zero pressure.
- `set_uniform_velocity(v, FaceSelection::All)`: sets every face. This is a test
  utility only.
- `set_uniform_velocity(v, FaceSelection::ActiveOnly, &faces)`: writes `0.0` to
  blocked faces. That is *not* an obstacle boundary condition.

**Divergence.** `compute_divergence(mac, mask)` returns a cell-centered field.
For each fluid cell:

```
div = (u(i+1,j,k) − u(i,j,k))/dx + (v(i,j+1,k) − v(i,j,k))/dy + (w(i,j,k+1) − w(i,j,k))/dz
```

- **Solid cells:** they hold `0.0`, with no physical meaning; a value is valid
  only where the mask is Fluid.
- **Face values:** used as stored. The operator never alters them.
- **Statistics:** `divergence_stats` returns max |div|, mean |div|, RMS and the
  fluid-cell count, or `std::nullopt` when there are no fluid cells.
- **Measuring only:** nothing in this milestone corrects divergence.

**Validation** (`engine/tests/test_mac_grid.cpp`, `test_divergence.cpp`):

- **Exact results:** zero and uniform fields give exactly 0. A linear field
  `(ax, by, cz)` gives `a+b+c` to 1e-12 with `dx=0.3, dy=0.7, dz=1.1`. The rotation
  field `(ay, −ax, 0)` gives less than 1e-13.
- **Stream function:** the discrete curl of an arbitrary stream function is
  divergence-free to roundoff (below 1e-11).
- **Truncation error:** the sampled field `u = sin x cos y, v = −cos x sin y` with
  `dx ≠ dy` converges at second order:

| N | max \|div\| | observed order |
|---|---|---|
| 8 | 2.22e-3 | – |
| 16 | 5.58e-4 | 1.989 |
| 32 | 1.40e-4 | 1.997 |
| 64 | 3.50e-5 | 1.999 |

- **Masks:** face classification is checked against brute-force cell adjacency
  on a voxelized sphere and on the procedural box.

## MAC boundary conditions (approved for Milestone 3B)

The code is in `engine/include/aeroflow/boundary.hpp` and `src/boundary.cpp`.
It is not used by the CLI or UI yet; it is the interface for the pressure
projection and time stepping to come.

**Model** (fixed walls; only `U_in` is configurable, default 5 m/s, finite and ≥ 0):

| Boundary | Normal velocity | Tangential velocity | Pressure |
|---|---|---|---|
| Inlet, x = 0 | `u = U_in` (Dirichlet) | 0 (odd reflection) | Neumann |
| Outlet, x = Lx | pressure-controlled; predictor `u(Nx) = u(Nx−1)` | zero gradient (even reflection) | **Dirichlet p = 0 on the outlet plane** |
| y = 0, Ly, z = 0, Lz | 0 (free-slip) | zero normal derivative (even reflection) | Neumann |
| Obstacles (mesh or box) | 0 (no penetration, stationary) | 0, no-slip (odd reflection) | Neumann |

**Face roles.** `classify_boundaries(mask)` reuses `classify_faces` and the face's
position to give every `u`, `v` and `w` face exactly one `VelocityFaceBC`:
`Interior`, `Inlet`, `Outlet`, `SlipWall`, `SolidWall` or `Inactive`.

- **No corner conflicts for normal velocity:** a face has one normal direction, so
  it lies on at most one wall (`u` only on the x walls, `v` on y, `w` on z).
- **Solids win:** where a solid touches a tunnel wall, the face is `Inactive`, not
  inlet, outlet or slip wall.
- **Fully solid domain or blocked outlet:** rejected by `validate_boundaries`.
  With no `Outlet` face, the pressure has no reference and inflow cannot leave.

**Applying velocities.** `apply_velocity_bcs(mac, map, bc)` writes only
boundary-controlled faces.

- **Interior faces:** never modified.
- **Order:** prescribed faces are set first; then outlet faces copy the opposite
  face of their cell.
- **Idempotent:** applying it twice gives the same result.

**Tangential conditions.** No sample lies on a wall, so these conditions can't be
stored. Instead, `tangential_neighbor(mac, map, component, i, j, k, axis, side)`
returns the neighbor a future viscous or advective stencil needs, or a ghost value
mirrored about the boundary halfway between samples:

- **+value:** free-slip walls and the outlet.
- **−value:** the inlet (zero transverse velocity) and `Inactive` obstacle
  neighbors (no-slip).
- **Stair-step obstacles:** a neighbor face lying on an obstacle face
  (`SolidWall`) is returned as stored (0). At stair-step corners the wall is then
  taken at that sample, not halfway.
- **Accuracy:** no-slip is imposed on **voxel faces**, not on the CAD surface, so
  near curved bodies the wall position has an O(Δ) stair-step error. Cut-cell or
  immersed-boundary methods would be needed to remove it.

**Pressure for the projection.**

- **Unknowns:** one pressure per fluid cell.
- **`pressure_condition(bc)`:**
  - `Interior` faces are `Coupled`; the projection corrects their velocity.
  - `Outlet` faces are `Dirichlet`. The ghost is `outlet_ghost_pressure(p) = −p`,
    so p = 0 exactly on the outlet plane, and the velocity is corrected with it.
  - All prescribed faces are `Neumann`: left out of the Laplacian and never
    corrected, so the inlet, wall and obstacle values survive the projection.
- **Gauge:** fixed by the outlet Dirichlet faces.

**Diagnostics.** `boundary_fluxes` reports inlet and outlet volume flux, open
areas, and total leakage through walls and obstacles, which must be 0 after
`apply_velocity_bcs`.

**Not yet enforceable.**

- **Tangential and outlet derivative conditions:** they act only through ghost
  values, so they take effect when viscous and advective stencils exist.
- **Outlet mass balance:** the outlet velocity is final only after a projection.
  Without one, outflow equals inflow only for flows that are already
  divergence-free.
- **Convective outflow:** a convective (radiation) outflow condition needs a time
  step and is deferred.
- **Floor:** ground-plane (no-slip floor) runs are not supported. All four lateral
  walls are free-slip, matching the free-stream tunnel fitting from 2A.

## Numerical fields and visualization (approved for Milestone 4A)

### AFFIELD v1: canonical binary field file

There is one file per field per frame. The C++ engine writes it
(`engine/src/field_io.cpp`), FastAPI validates it and serves the same bytes
(`backend/app/fields.py`), and TypeScript decodes it (`frontend/src/lib/fields/format.ts`).

```
bytes 0..7     "AFFIELD\0"
bytes 8..11    uint32 LE format version (1)
bytes 12..15   uint32 LE header length H
bytes 16..     UTF-8 JSON header, space-padded so that 16 + H is a multiple of 8
blocks         at the absolute offsets in header.blocks (8-byte aligned, little-endian):
               "values" float32, "valid" uint8 (1 = valid sample)
```

| Header key | Meaning |
|---|---|
| `name`, `quantity`, `long_name`, `units` | Unique id (`divergence_diagnostic`), physical quantity, label, units (`1/s`) |
| `signed` | Quantity may be negative, so the viewer uses a zero-centered diverging map; nonnegative quantities get a sequential map |
| `kind`, `components` | `scalar`, 1 (vector components are separate staggered scalar fields, see below) |
| `location` | `cell`, `face_x`, `face_y` or `face_z` |
| `shape`, `index_order` | Array extents `[n0,n1,n2]` and `i + n0*(j + n1*k)`, always x-fastest and never transposed |
| `grid` | `resolution`, `origin` `[0,0,0]`, `spacing` `[dx,dy,dz]`, `domain` `[Lx,Ly,Lz]` (m) |
| `frame` | `{index, time_s, label}`; `index` is in the file name `<name>.f<index>.affield` |
| `mask`, `provenance` | How validity was derived; `{kind: "diagnostic" | "solution", solved, description, ...}` |
| `stats` | `valid_count`, `min`, `max`, `max_abs`, `mean_abs`, `rms` over valid samples, computed in **float64** before the values are narrowed to float32. All are `null` when there are no valid samples |

Values are transported as float32, half the size of float64 (8.4 MB at the
128³ job limit), and stats stay exact. `fixtures/fields/golden_v1.affield` is a
golden file written by C++ and checked byte for byte by the C++, Python and
TypeScript tests. Regenerate it only for an intentional format change (and bump
the version) with `AEROFLOW_UPDATE_GOLDEN=1 engine/build/aeroflow_tests`.

### Divergence diagnostic (`divergence_diagnostic`)

**This is a measurement of an initialized field, not a CFD solution.**

- **Velocity:** a MAC grid is built on the solid mask. `u = 5 m/s` on active
  x-faces, which are `Fluid` and `DomainBoundary` faces, so the inflow and outflow
  tunnel walls carry 5 m/s. `v = w = 0`, and faces blocked by solids
  (`SolidBoundary`, `SolidInterior`) carry 0.
- **Divergence:** computed at fluid-cell centers; solid cells are invalid.
- **Without an obstacle:** the field is exactly divergence-free.
- **With an obstacle:** fluid cells just upstream of an obstacle face read
  `−5/dx` (sink), and cells just downstream read `+5/dx` (source). Everything else
  is 0.

### Backend

- **Requesting fields:** `POST /api/grid-jobs` with `"fields": ["divergence_diagnostic"]`.
  This is opt-in; the default is none.
- **Job response:** the job's `fields` list holds the validated headers (without
  block offsets). Large arrays never appear in JSON.
- **Endpoints:**
  - `GET /api/grid-jobs/{id}/fields` lists field metadata.
  - `GET /api/grid-jobs/{id}/fields/{name}?frame=0` returns the raw AFFIELD bytes
    (`application/octet-stream`, `X-AeroFlow-Field-Version: 1`).
  - Errors: `404` if the job or field doesn't exist (the message lists what is
    available), `410` if the data was removed.
- **Validation:** magic, version, JSON header, shape against location and grid,
  dtypes, block bounds and lengths.
- **Storage:** files live in `data/jobs/<job_id>/` while the job stays in memory.
  They are deleted when the job is evicted (after the last 50), on failure, and at
  startup.
- **Limits:**

  | Variable | Default | Purpose |
  |---|---|---|
  | `AEROFLOW_MAX_FIELD_MB` | 64 | Per-file limit, checked against an estimate before the engine runs (`422`) |
  | `AEROFLOW_MAX_FIELDS_DISK_MB` | 1024 | Total disk budget (`507` when full) |

### Frontend

All field code is generic; nothing is specific to divergence except the export
checkbox.

- **`lib/fields/format.ts`:** types, `decodeField` (validates everything and
  reports precise errors), and `encodeField`.
- **`lib/fields/sampling.ts`:** sample positions for every location,
  `extractSlice(field, axis, index)`, and the physical plane corners.
  - **Positions:** a sample sits at `origin + (idx + offset)·d`, where the offset
    is ½ on cell axes and 0 on a face field's staggered axis.
  - **Footprint:** each sample covers `[p − d/2, p + d/2]`. A cell-field slice
    therefore spans exactly `[0, L]`, and a face field's `[−d/2, L + d/2]`.
  - **Slice axes:** x-slices use (u, v) = (y, z), y-slices (x, z), z-slices (x, y).
- **`lib/fields/colormap.ts`:** a diverging map (blue / grey / red, centered at 0,
  default range ±max|v|) and a sequential map (viridis). Values are normalized and
  clamped, and a degenerate range maps to the neutral color. Invalid samples become
  transparent RGBA.
- **`components/FieldSlice.tsx`:** **one quad with a `DataTexture`** per slice
  (RGBA8, nearest filtering), with one texel per sample and no interpolation. A
  512×512 slice is a single texture upload.
- **`components/FieldLegend.tsx` and `components/FieldsPanel.tsx`:** the legend
  (units, range, slice position, frame, diagnostic badge) and the controls (field
  selector, X/Y/Z slice, index, auto or manual range, opacity, stats, provenance).

### Extending to future solver output

- **Pressure:** a `cell` field (`signed: true`, Pa) using the same renderer.
- **Staggered velocity:** exported unchanged as three scalar fields, `u`
  (`face_x`, shape `(Nx+1)×Ny×Nz`), `v` (`face_y`) and `w` (`face_z`), each with
  its own active-face `valid` block. They are never resampled into the file.
- **Derived cell quantities:** computed **in C++** and exported as new `cell`
  fields with their own provenance.
  - **Velocity magnitude:** `|U| = sqrt(ū² + v̄² + w̄²)` with face-pair averages
    `ū = (u(i)+u(i+1))/2` (and likewise for `v̄`, `w̄`), `signed: false`.
  - **Vorticity magnitude:** from MAC differences averaged to cell centers.
  - **Documentation:** the exact stencils will be written down when these fields
    are implemented.
- **Vector arrows and streamlines:** would consume the cell-averaged velocity
  fields, or the face fields directly. The AFFIELD format already describes both.
- **Time frames:** `frame.index` and `time_s` in the header, files named
  `<name>.f<frame>.affield`, and the `?frame=` query parameter. Animation only
  needs to fetch more frames; the format and storage stay the same.

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

1. **Model:** choose an `.stl` or `.obj` file and set **Model units** (default:
   millimeters). The panel shows the size from the file and the physical size in
   meters after the transform.
2. **Transform:** move, rotate, or scale the model with the on-screen gizmo or
   the numeric fields.
3. **Fit Tunnel to Model:** set the clearance multipliers, then check and apply
   the proposed tunnel and grid. If it's over budget, pick the coarser grid or
   **Adjust manually**. Use **Display → Reframe view** to re-center the camera.
4. **Wind tunnel / Grid resolution:** set `Lx, Ly, Lz` and `nx, ny, nz`. The
   lattice preview updates live.
5. **Solid geometry source:** choose **Procedural box** (set the test box) or
   **Imported mesh** (voxelize the uploaded model with its current units and
   transform). A warning appears if the model is poorly resolved; the grid is
   never refined automatically.
6. **Generate grid:** the backend runs the engine, and the solid cells are
   drawn in orange while the imported mesh stays visible. The results show the
   source, the solid and fluid counts, the spacing and, for meshes, the
   voxelization time and volume error. Use **Display → Slice** to step through
   one cell layer at a time.

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

Use `"mesh": {"triangles_file": "<path>.aftri", "bvh_leaf_size": 4}` instead of
`test_box` to voxelize a mesh; setting both is an error. `test_box` is optional. A mesh
job's output also has `geometry_source`, a `mesh` block (validation, bounds,
volume, volume error) and `voxelization` statistics. An invalid mesh exits with
code `2` and error code `invalid_mesh`. The output JSON contains `grid` (resolution, domain,
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
- `TriangleMesh`, `read_triangle_file`, `validate_mesh`, `signed_volume`
  (`mesh.hpp`); `Bvh` (`bvh.hpp`); `intersect_ray_triangle` (`ray.hpp`);
  `voxelize_mesh` (`voxelize.hpp`).
- `build` and `grid_metadata` (`metadata.hpp`): CLI output.

## Backend API

| Method | Path | Description |
|---|---|---|
| GET | `/api/health` | Service status and engine availability/version |
| GET | `/api/limits` | Grid budget (`max_cells`, `max_cells_per_axis`), upload limit and `max_triangles` |
| POST | `/api/models` | Multipart upload (`file`) of `.stl`/`.obj`; validates structure and returns `model_id` and counts |
| GET | `/api/models/{model_id}` | Uploaded model metadata |
| POST | `/api/grid-jobs` | Validates the grid config, runs the engine, and returns the job with the result. `geometry_source`: `procedural_box` (default, uses `test_box`) or `mesh` (requires `model_id` and `model_transform`; `test_box` must be null) |
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
| `AEROFLOW_MAX_TRIANGLES` | `1000000` | Triangle limit for mesh voxelization |
| `AEROFLOW_ENGINE_TIMEOUT_S` | `60` | Engine subprocess timeout |

Interactive docs are served at http://127.0.0.1:8000/docs.

## Tests

```bash
make test                     # everything
ctest --test-dir engine/build --output-on-failure        # C++: grid, mesh, ray/BVH, voxelization, MAC grid, divergence, BCs, AFFIELD
cd backend && .venv/bin/python -m pytest -q              # API validation, uploads, mesh parsing, engine round-trips
cd frontend && npm test && npm run build                 # TS unit tests + typecheck + production build
```

The backend tests that run the engine are skipped if `engine/build/aeroflow-grid`
has not been built.

## Not implemented yet

These are left out deliberately, and each will need decisions from the
developer:

- Volume fractions or cut cells for partially filled cells. The binary mask stays
  as it is, and a fraction field would be added alongside it.
- The Navier–Stokes solver: discretization, variable arrangement, time
  integration, pressure solve
- Boundary conditions: inlet, outlet, walls, and solid surfaces
- Non-uniform or refined grids
- Advection, diffusion, pressure projection and time stepping (which will consume
  the 3B boundary operators)
- Pressure, velocity, vorticity, vector-arrow, streamline and multi-frame renderers
  (the field format and pipeline already support them)
