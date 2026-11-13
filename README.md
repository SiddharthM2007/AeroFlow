# AeroFlow

A browser-based 3D virtual wind tunnel. Users import STL/OBJ models, configure
the tunnel and computational grid, and (in later milestones) run a custom C++
incompressible Navier–Stokes solver.

**Current status: Milestone 7, pressure solver acceleration.** It builds on
Milestone 1 (the grid foundation), 2A (geometry-aware tunnel fitting), 2B (CAD
mesh voxelization), 3A (staggered MAC grid and discrete divergence), 3B
(boundary conditions), 4A (field visualization), 5 (pressure projection) and 6
(time-dependent Navier–Stokes).

- **Pressure solver (7):** a multigrid-preconditioned conjugate gradient solver
  (MG-PCG) solves the same pressure equation to the same tolerance as the
  original Jacobi-PCG, about 7–19× faster per solve on 65 k–2 M cell grids.
  `auto` (the default) selects it from 1,000 fluid cells; Jacobi-PCG remains
  selectable and is the automatic fallback.

- **Navier–Stokes (6):** a first-order fractional-step solver advances the flow
  in time. Runs execute as background jobs with progress, cancellation and a
  preflight cost check, and write one field file per frame. The viewer has a
  frame slider with the physical time.

- **Field pipeline:** numerical fields now travel from C++ through FastAPI to the
  browser in a versioned binary format (AFFIELD).
- **Viewer:** fields are shown as colored 3D slices with a legend.
- **Boundary conditions (3B):** the C++ MAC grid has reusable boundary-condition
  operators for the future solver.
- **Pressure projection (5):** a matrix-free pressure Poisson solve projects an
  initialized velocity field to an approximately divergence-free one. Pressure,
  divergence and velocity magnitude can be viewed in the browser.
- **First field:** a *divergence diagnostic* of an initialized velocity field.
  It, and the single projection of Milestone 5, are labelled in the viewer so
  they are not mistaken for the time-dependent solution.

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

## Pressure Poisson solver and velocity projection (approved for Milestone 5)

The code is in `engine/include/aeroflow/pressure.hpp` and `src/pressure.cpp`.

**Unknowns and formulation.** The solve works in the projection potential
`φ = (Δt/ρ)·p`, with one unknown per fluid cell numbered x-fastest. Solid cells
are not unknowns. The reported pressure is `p = (ρ/Δt)·φ` in Pa, a gauge
pressure with p = 0 on the outlet plane.

**Gradient G**, with h the spacing along the face normal:

| Face (3B role) | `(Gφ)_f` | Velocity corrected? |
|---|---|---|
| `Interior` (Coupled) | `(φ_R − φ_L)/h` | yes |
| `Outlet` (Dirichlet) | ghost `φ_g = −φ_L`, giving `(φ_g − φ_L)/dx = −φ_L/(dx/2)`: p = 0 **on the outlet plane**, half a cell from the last center | yes |
| `Inlet`, `SlipWall`, `SolidWall`, `Inactive` (Neumann) | 0 | **never** |

**Projection and pressure system.** The projection is `u_new = u* − Gφ` on
Coupled and Outlet faces only. With the existing MAC divergence D,
`L = D∘G` is symmetric negative semidefinite, so the solver works with the SPD
operator **`A = −L`**:

```
(Aφ)_c = Σ_coupled (φ_c − φ_nb)/h²  +  Σ_outlet 2φ_c/dx²        A φ = b,   b = −D u*
```

- **Remaining divergence:** `D u_new = −(b − Aφ)`. The divergence left after
  projection is exactly the negated solver residual. It is recomputed
  independently with `compute_divergence` and never set to zero.
- **Operator:** matrix-free, with no stored matrix. Per fluid cell it keeps the
  cell index, a 6-bit coupled-face mask, the diagonal and its inverse, and the
  component id.

**Solver.** Jacobi-preconditioned conjugate gradient in double precision.

- **Stopping rule:** `‖b − Aφ‖₂ ≤ tol·‖b‖₂` (default `tol = 1e-10`,
  `max_iterations = 10000`), checked with the **true** residual. The solver
  restarts if the recursively updated residual has drifted.
- **Memory:** work vectors are allocated once, with no allocation inside the
  iteration loop.
- **Zero right-hand side:** a right-hand side of exactly 0 (uniform flow in an
  empty tunnel) finishes in 0 iterations, and the velocity is unchanged bit for
  bit.
- **Failures:** CG breakdown gives `solver_breakdown`, and not reaching the
  tolerance gives `projection_not_converged`. **Either one fails the job.**

**Disconnected fluid regions.** Components are found by flood fill through
coupled faces.

| Component | Handling |
|---|---|
| Touches the outlet | SPD, solved directly |
| No outlet, net source ≈ 0 (e.g. a cavity enclosed by a mesh) | Singular; solved with a **zero-mean pressure gauge** |
| No outlet but a net source (e.g. a pocket fed by the inlet) | **`incompatible_component` error** that names the region; inflow cannot leave |

**Settings** (engine `"projection"` block, or `projection` in a grid-job request):

| Setting | Default | Notes |
|---|---|---|
| `inlet_velocity` | 5 m/s | also used by the 4A diagnostic |
| `density` | 1.225 kg/m³ | |
| `dt` | `0.5·min(dx,dy,dz)/U_in` (CFL 0.5) | required when `U_in = 0` |
| `tolerance` | 1e-10 | relative residual, in (0, 1) |
| `max_iterations` | 10000 | 1 to 1,000,000 |

Invalid values are rejected with field-level `422` errors (backend) or exit code
2 (engine).

**Fields.** These are opt-in AFFIELD ids, all produced by one projection of the
initialized field (uniform `U_in` on active faces, then the 3B boundary
conditions). Their provenance is `kind: "projection", solved: false`.

| Field | Units | Map |
|---|---|---|
| `pressure` | Pa | diverging |
| `divergence_before` | 1/s | diverging |
| `divergence_after` | 1/s | diverging |
| `velocity_magnitude` | m/s | sequential; `√(ū²+v̄²+w̄²)` from face-pair averages |

The job result also gains a `projection` block: iterations, relative residual,
divergence before and after, inlet and outlet flux, wall leakage, Δt, memory and
timings. The UI shows it in the Fields panel.

**Validation** (`engine/tests/test_pressure.cpp`, `backend/tests/test_projection.py`):

- **Operator consistency:** `Aφ = D(−Gφ)` to 1e-12 on an anisotropic obstacle
  grid; A is symmetric and positive definite.
- **Half-cell outlet:** a 1D problem with p = 0 on the plane has the discrete
  solution `a(x² − Lx²)/2 − a dx²/8` exactly, and the error ratio under
  refinement is 4.000 (second order). Pinning φ at the last cell center would be
  first order.
- **Projection results:**
  - Uniform empty tunnel: unchanged.
  - Random divergent field: RMS div 28.1 → 2.3e-9.
  - Voxelized sphere: max |div| 50 → 1.8e-9 with zero leakage, `Q_out = Q_in`,
    and prescribed faces bit-identical before and after.
- **Error cases:** non-convergence, incompatible regions, enclosed cavities and
  single-cell pockets.

**Performance** (Apple Silicon, single thread, full engine run including field
export; procedural box obstacle, tol 1e-10):

| Grid | Cells | PCG iterations | Solve | Peak RSS |
|---|---|---|---|---|
| 64×32×32 | 65,536 | 279 | 0.11 s | 13 MB |
| 128×64×64 | 524,288 | 561 | 1.8 s | 91 MB |
| 128³ (cap) | 2,097,152 | 790 | 10.8 s | 391 MB |
| 256×96×85 (anisotropic) | 2,088,960 | 1,216 | 15.8 s | 406 MB |

Jacobi PCG needs roughly O(N^(1/3)) iterations, so a full-size grid takes 10–16 s.
That is inside the 60 s engine timeout but slow. Multigrid or a parallel operator
are the natural next optimizations.

**Limitations.**

- **Single projection:** this is one projection of a prescribed initial field.
  There is no advection, diffusion or time stepping, so the flow is a
  potential-flow-like, inviscid correction, not a Navier–Stokes solution.
- **Pressure magnitude:** it scales as ρ/Δt (an impulse pressure), not a
  steady aerodynamic pressure.
- **Obstacle shape:** no-slip enters only through no penetration at voxel faces;
  stair-step geometry remains.

## Time-dependent Navier–Stokes (approved for Milestone 6)

The engine solves the incompressible equations

    ∂u/∂t + (u·∇)u = −∇p/ρ + ν∇²u,   ∇·u = 0

on the MAC grid with constant ρ and ν, using the 3B boundary conditions and the
Milestone 5 projection.

**One step** (`Simulation::advance`, `engine/src/simulation.cpp`):

1. Apply the velocity boundary conditions (prescribed inlet, slip walls,
   zero-gradient outlet, no-penetration solid faces).
2. Predictor, forward Euler, on `Interior` faces only:
   `u* = uⁿ + Δt [−(uⁿ·∇)uⁿ + ν∇²uⁿ]`. All other faces are copied unchanged.
3. Projection: solve `A φ = −D u*` with Jacobi PCG, then `uⁿ⁺¹ = u* − Gφ`
   and `p = ρφ/Δt`.
   - **Warm start:** the solve starts from the previous step's pressure,
     `φ_guess = (Δt/Δt_prev) φ_prev = Δt p_prev/ρ`. It stays consistent when a
     step is shortened to land on a frame.
   - **First step:** starts cold. The initial projection's potential is an
     impulse, not a pressure.
   - **Accuracy:** the stopping test is relative to ‖b‖, so the warm start
     changes only the iteration count.
4. Check for non-finite values and record step statistics.

**Spatial operators** (`engine/src/momentum.cpp`):

- **Advection:** advective form with first-order upwinding. Each derivative
  takes the one-sided difference on the upstream side of the transporting
  velocity. The transverse velocities at a face are 4-point averages.
- **Diffusion:** the 7-point Laplacian on each face grid.
- **Neighbours across boundaries:** they use the 3B ghost rules: even at slip
  walls and the outlet, odd (no-slip) at inactive solid faces and the inlet,
  and the stored value at solid-wall faces. Along a face's own axis past the
  outlet, the value is copied (zero gradient).

**Time step.** Forward Euler with upwinding is stable (positive coefficients)
when

    Δt · ( |u|max/dx + |v|max/dy + |w|max/dz + 2ν(1/dx² + 1/dy² + 1/dz²) ) ≤ 1

The solver uses `Δt = CFL × that limit` (CFL ≤ 1, default 0.5). It recomputes
the limit every step from the current velocities, and shortens steps so frames
land exactly on their target times.

- **User Δt:** a user Δt acts as an upper cap. If it exceeds the stability limit
  of the projected initial field, the job fails with `unstable_timestep`. It is
  never silently reduced.
- **Step budget:** if the run would need more than `max_steps`, it is rejected
  before starting (`step_budget`).

**Pressure.** The scheme is non-incremental (Chorin): each step solves for the
full pressure, with no pressure in the predictor.

- `p` is a gauge pressure, with p = 0 on the outlet plane.
- It is only first-order accurate in time, O(Δt). Expect a thin
  boundary-layer error near walls.
- Unlike the Milestone 5 impulse pressure, it has physical units and a physical
  magnitude.

**Initial field.**

- `uniform`: U_in everywhere, projected once.
- `rest`: zero velocity inside, with only the inlet prescribed.

**Settings** (`simulation` block of a grid job):

| Setting | Default | Notes |
|---|---|---|
| `inlet_velocity` | 5 m/s | |
| `density` | 1.225 kg/m³ | |
| `viscosity` | 1.5e-5 m²/s | ν ≥ 0 |
| `duration` | 0.5 s | |
| `frame_interval` | 0.05 s | ≤ 10,000 frames |
| `cfl` | 0.5 | (0, 1] |
| `dt` | adaptive | optional cap |
| `max_steps` | 2000 | |
| `initial` | `uniform` | |

The pressure tolerance and iteration limit are shared with Milestone 5.

**Output.**

- **Fields:** `sim_pressure`, `sim_velocity_magnitude` and `sim_divergence` at
  cell centres; `sim_u`, `sim_v` and `sim_w` on their faces.
- **Files:** one AFFIELD file per field per frame (`<name>.f<frame>.affield`).
  Each header carries the frame time, the step and
  `provenance.kind = "simulation"`.
- **Summary:** the job result contains a simulation summary with status, steps,
  frame times, wall time, PCG iterations, memory and per-step history (Δt,
  residual, divergence, inflow/outflow, wall leak, max speed, kinetic energy).

**Budgets.** Three guards keep a run bounded:

- **Preflight:** the backend estimates steps, runtime, output size and memory
  (`backend/app/simulation_estimate.py`, calibrated on the benchmarks below).
  It rejects a run before it starts (422) when any of these exceeds its budget.
- **Wall-time stop:** the engine stops cleanly at the wall-time budget
  (`AEROFLOW_SIM_TIMEOUT_S`). It keeps the frames already written, and the job
  ends as `stopped` with error code `wall_time_budget`.
- **Cancellation:** cancelling kills the run and discards its frames.

**Validation** (`engine/tests/test_momentum.cpp`, `test_simulation.cpp`):

- **Exactness:**
  - The operators are exact for uniform flow and linear advected profiles.
  - The Laplacian of a quadratic is exact on anisotropic spacing.
  - Boundary neighbours are identical to the 3B `tangential_neighbor`.
- **Upwind transport of a smooth profile:** observed order 0.82 → 0.88 → 0.94
  under refinement (first order).
- **Viscous decay of a sine mode:** matches the discrete prediction to round-off,
  and the continuous decay `exp(−νk²t)` to 8e-5.
- **Uniform flow and fluid at rest:** both are preserved bit-exactly.
- **Box obstacle:** `wall_leak = 0` and `Q_out = Q_in` to 1e-6 every step;
  max |div| after projection is at the solver tolerance.
- **Δt refinement:** the error ratio is 2.05 (first order in time).
- **Enclosed cavity without inflow:** kinetic energy never increases.
- **Rejections:** invalid settings, unstable Δt and step budgets are rejected.
  Frames land exactly on their times, and the wall-time budget stops a run
  cleanly.

**Performance** (Apple Silicon, single thread, box obstacle, 0.1 s simulated,
3 frames × 3 fields):

| Grid | Cells | Steps | Time/step | PCG it/step (mean / max) | Peak memory | Output |
|---|---|---|---|---|---|---|
| 64×32×32 | 65,536 | 66 | 97 ms | 212 / 285 | 19 MB | 2.8 MB |
| 128×64×64 | 524,288 | 152 | 1.56 s | 414 / 531 | 141 MB | 22.5 MB |

The pressure solve dominates, and warm starts cut iterations only modestly. The
default 0.5 s run takes about 30 s at 64×32×32 and about 20 min at 128×64×64.
At the 2.1 M-cell cap it would take hours, so the preflight rejects it under the
default 15-minute budget. Multigrid is the natural next optimization.

**Limitations.**

- **Numerical viscosity:** first-order upwinding adds roughly `|u|h/2`. At
  practical grid sizes this far exceeds air's ν, so the effective Reynolds
  number is grid-dependent and much lower than the physical one. Wakes are
  overly diffused.
- **Time accuracy:** first order in time (forward Euler, non-incremental
  pressure). The pressure has an O(Δt) error.
- **Outlet:** the zero-gradient outlet does not prevent backflow. A wake that
  reaches the outlet can re-enter.
- **Geometry and resolution:** no turbulence model; stair-step voxel geometry;
  uniform grid.

## Pressure solver acceleration (approved for Milestone 7)

Both solvers are preconditioned conjugate gradients on the unchanged Milestone 5
operator `A = −D∘G`.

- **Shared:** the same right-hand side, the same boundary conditions (Neumann on
  prescribed faces, p = 0 on the outlet plane), the same compatibility check and
  zero-mean gauge for enclosed regions, and the same stopping test.
- **Stopping test:** the true relative residual ‖b − Aφ‖ ≤ tol·‖b‖. Failure is
  still `projection_not_converged`.
- **Only the preconditioner differs.**

| Setting | Behaviour |
|---|---|
| `jacobi_pcg` | z = D⁻¹r (the validated Milestone 5 baseline) |
| `mgpcg` | z = Π·B·Π r: one symmetric multigrid V-cycle B, where Π removes the mean of every enclosed (no-outlet) region, i.e. A's nullspace |
| `auto` (default) | `mgpcg` from 1,000 fluid cells, else `jacobi_pcg` |

`pressure_solver` is accepted in the `projection` and `simulation` blocks of a
grid job (API and engine config) and in the viewer's Pressure solver menu. The
choice changes speed only, not the physics.

**Multigrid V-cycle** (`engine/src/multigrid.cpp`):

- **Finest level:** the PressureSystem operator itself. A test checks bit-identical
  results.
- **Coarse grids:**
  - Cell-centred 2:1 coarsening; an odd count n becomes ⌈n/2⌉ with a half-width
    last cell.
  - Semi-coarsening: an axis is coarsened only while its width is ≤ 1.5 × the
    smallest width.
  - A coarse cell is fluid if any child is.
- **Coarse operators:** rediscretized in conductance form, with each coarse
  face's coefficient = (summed open area of the finer faces) / (true
  centre-to-centre distance), and outlet faces using half the cell width.
  - They are symmetric and positive semidefinite by construction, with partial
    cells exact.
  - Obstacles appear as partially open faces.
  - Regions separated by walls thinner than a coarse cell may merge on coarse
    levels. That only affects the preconditioner's quality.
- **Transfers:** piecewise-constant (default) or trilinear; restriction is the
  exact transpose (R = Pᵀ).
- **Cycle:** V(2,2) red-black Gauss-Seidel, pre-sweeps red→black and post-sweeps
  black→red (the adjoint ordering), with a fixed 16 + 16 symmetric sweeps on the
  coarsest level (≤ 512 unknowns).
- **Properties:** B is a fixed linear map, symmetric and positive definite in the
  PCG inner product. Tests check this, including dense Cholesky factorizations.
- **Reuse:** the hierarchy is built once per grid and obstacle configuration and
  reused, with its work buffers, by every time step.

**Fallback.** If the hierarchy cannot be built, or rᵀz ≤ 0, non-finite, or
dᵀAd ≤ 0 on the multigrid path, the solve continues with Jacobi-PCG from the
current iterate. The switch is reported in the job summary and logged as a
warning; it is never silent.

**Warm starts** carry the pressure, not the potential, across steps:
φ_guess = (Δt/Δt_prev)·φ_prev. The first physical step starts cold, because the
initial projection's potential is an impulse.

**Optional max-divergence test** (engine setting `max_divergence`, off by
default): it also requires max |b − Aφ| ≤ the given value (1/s).

- **Why MG-PCG's maximum is higher:** it leaves its (smaller) residual
  concentrated next to obstacle surfaces, where Jacobi-PCG spreads it.
  - RMS: 5–10× lower with MG-PCG.
  - Max/RMS: up to 223 with MG-PCG vs 5–8 with Jacobi-PCG.
- **Cost of the test:** about 3% (box) to 11% (STL) of solve time.
- **Why it is off:** the 2-norm test is the established criterion, and the
  difference (≤ 5e-7 1/s) is physically negligible.

**Benchmarks** (Apple M2 Pro, single thread, tolerance 1e-8, the same steps with
both solvers, including the initial projection and setup):

| Case | Steps | Total time | Pressure time per step | Iterations per step | MG setup / memory |
|---|---|---|---|---|---|
| 64×32×32 box | 20 | 2.58 → 0.46 s | 114 → 14 ms | 271 → 6.6 | 2 ms / 1.5 MB |
| 64×32×32 Valve Body STL | 20 | 2.68 → 0.53 s | 119 → 18 ms | 282 → 8.1 | 2 ms / 1.6 MB |
| 128×64×64 box | 8 | 19.4 → 1.76 s | 2047 → 131 ms | 545 → 7.0 | 14 ms / 12 MB |
| 128×64×64 Valve Body STL | 8 | 20.5 → 2.36 s | 2208 → 199 ms | 595 → 10.9 | 15 ms / 12 MB |
| 200×102×102 box | 4 | 66.9 → 4.65 s | 12.5 → 0.66 s | 851 → 8.0 | 64 ms / 49 MB |
| 200×102×102 Valve Body STL | 4 | 65.5 → 5.82 s | 12.0 → 0.87 s | 838 → 12.0 | 62 ms / 49 MB |
| 256×96×85 anisotropic box | 3 | 58.4 → 4.33 s | 13.9 → 0.75 s | 873 → 8.3 | 106 ms / 71 MB |

- **Agreement:** final velocities agree with Jacobi-PCG to ≤ 4e-10 and pressures
  to ≤ 2e-8 (relative).
- **Conservation:** wall leakage is 0, |Q_out/Q_in − 1| ≤ 2e-10, and no run fell
  back.
- **Defaults chosen by measurement:** V(1,1) and trilinear transfers were both
  slower overall.
- **Benchmark tool:** `pressure_bench --simulate compare` reproduces these
  numbers.

**Preflight estimates** (`backend/app/simulation_estimate.py`) use a separate
calibrated model per solver, and `auto` is estimated with the solver it will
select:

- **Jacobi-PCG:** 7·N^(1/3) iterations × 7.5 ns per cell-iteration.
- **MG-PCG:** 12 iterations × 36 ns per cell-iteration, plus setup.
- **Other step work:** 120 ns per cell.

A run rejected over the time budget is told whether the other solver would fit.

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
| GET | `/api/grid-jobs/{job_id}` | Fetch a job (kept in memory); poll this for simulation progress |
| POST | `/api/grid-jobs/estimate` | Preflight estimate for a simulation request (steps, Δt, runtime, output, memory, problems); nothing is run |
| POST | `/api/grid-jobs/{job_id}/cancel` | Cancel a queued or running simulation (409 if already final) |
| GET | `/api/grid-jobs/{job_id}/fields/{name}?frame=N` | AFFIELD file for one field and frame |

Grid jobs without simulation fields run synchronously, because the engine
finishes in milliseconds to seconds. Jobs that request `sim_*` fields return
`201` immediately with `status: "queued"` and an `estimate`. They run one at a
time in the background, through `running` with live `progress`, and end as
`completed`, `stopped` (wall-time budget; frames kept), `failed` or `cancelled`
(frames removed). Jobs that have not finished are never evicted. They are stored in memory, and only the last 50 are kept. If the
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
| `AEROFLOW_SIM_TIMEOUT_S` | `900` | Wall-time budget per simulation (preflight + engine hard stop) |
| `AEROFLOW_SIM_MAX_MEMORY_MB` | `4096` | Preflight memory limit per simulation |

Interactive docs are served at http://127.0.0.1:8000/docs.

## Tests

```bash
make test                     # everything
ctest --test-dir engine/build --output-on-failure        # C++: grid, mesh, ray/BVH, voxelization, MAC grid, divergence, BCs, AFFIELD, pressure, momentum, simulation
cd backend && .venv/bin/python -m pytest -q              # API validation, uploads, mesh parsing, engine round-trips
cd frontend && npm test && npm run build                 # TS unit tests + typecheck + production build
```

Pressure-solver benchmark (optional, not part of `make test`):

```bash
cmake -S engine -B engine/build -DAEROFLOW_BUILD_BENCH=ON && cmake --build engine/build
engine/build/pressure_bench --grid 128 64 64 --geometry box --steps 12   # also: empty | rest | mesh --mesh FILE.aftri
engine/build/pressure_bench --grid 128 64 64 --geometry box --steps 8 --simulate compare   # Jacobi-PCG vs MG-PCG
engine/build/pressure_bench --grid 128 64 64 --geometry box --multigrid                    # hierarchy, V-cycle profile
```

Each step solves the same system three ways (cold, legacy unscaled warm start,
Δt-rescaled warm start) and reports iterations, time, divergence, flux balance
and peak memory.

The backend tests that run the engine are skipped if `engine/build/aeroflow-grid`
has not been built.

## Not implemented yet

These are left out deliberately, and each will need decisions from the
developer:

- Volume fractions or cut cells for partially filled cells. The binary mask stays
  as it is, and a fraction field would be added alongside it.
- Higher-order advection (e.g. a TVD or QUICK scheme), second-order time
  integration and an incremental pressure scheme
- Multithreading (the multigrid smoother is red-black ordered and parallelizes
  naturally)
- A convective outlet or backflow treatment, and turbulence modelling
- Non-uniform or refined grids
- Vorticity, vector-arrow and streamline renderers, and frame animation
  (playback)
