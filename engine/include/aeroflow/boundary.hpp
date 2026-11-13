#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "aeroflow/grid3d.hpp"
#include "aeroflow/mac_grid.hpp"

namespace aeroflow {

// Boundary conditions for the staggered MAC grid (Milestone 3B, approved model).
//
// Tunnel walls (fixed model):
//   x = 0   inlet    u = U_in (Dirichlet); zero transverse velocity
//   x = Lx  outlet   pressure p = 0 at the outlet plane (Dirichlet); u is pressure-controlled,
//                    with a zero-gradient predictor value u(Nx) = u(Nx-1)
//   y = 0, Ly, z = 0, Lz   free-slip: zero normal velocity, zero normal derivative of tangential velocity
// Obstacles (voxelized mesh or procedural box): stationary no-slip, no penetration.
//
// Normal velocities live on faces, so each face has exactly one normal and lies on at most one
// wall: there are no corner/edge conflicts for normal components. A solid touching a tunnel wall
// takes precedence (the face is Inactive, velocity 0).
//
// Tangential conditions cannot be stored (no sample lies on a wall); they are ghost-value rules
// evaluated on demand by tangential_neighbor() for future viscous/advective stencils.

class BoundaryError : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

struct BoundaryConditions {
  double inlet_velocity = 5.0;  // m/s along +x; must be finite and >= 0
};

// Boundary role of one velocity face (same layout as MacGrid::u/v/w).
enum class VelocityFaceBC : std::uint8_t {
  Interior = 0,   // fluid|fluid: unknown; never written by boundary operators
  Inlet = 1,      // x = 0 face of a fluid cell: u = U_in
  Outlet = 2,     // x = Lx face of a fluid cell: pressure-controlled (zero-gradient predictor)
  SlipWall = 3,   // y/z tunnel-wall face of a fluid cell: normal velocity 0
  SolidWall = 4,  // obstacle face (fluid|solid): normal velocity 0
  Inactive = 5,   // solid|solid, or a tunnel-wall face covered by a solid: 0, never an unknown
};

// Faces whose value apply_velocity_bcs() fixes (Inlet, SlipWall, SolidWall, Inactive).
[[nodiscard]] constexpr bool is_prescribed(VelocityFaceBC bc) noexcept {
  return bc != VelocityFaceBC::Interior && bc != VelocityFaceBC::Outlet;
}

struct BoundaryMap {
  std::size_t nx = 0, ny = 0, nz = 0;
  std::vector<VelocityFaceBC> u;  // (nx+1) x ny x nz
  std::vector<VelocityFaceBC> v;  // nx x (ny+1) x nz
  std::vector<VelocityFaceBC> w;  // nx x ny x (nz+1)

  // Checked accessors (std::out_of_range).
  [[nodiscard]] VelocityFaceBC u_at(std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] VelocityFaceBC v_at(std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] VelocityFaceBC w_at(std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] std::size_t count(VelocityFaceBC bc) const noexcept;
  [[nodiscard]] bool matches(const MacGrid& mac) const noexcept;
};

// Assigns a boundary role to every face from the solid mask (via classify_faces) and the face's
// position: DomainBoundary u faces at i = 0 / Nx become Inlet / Outlet, DomainBoundary v and w
// faces become SlipWall, SolidBoundary -> SolidWall, SolidInterior -> Inactive, Fluid -> Interior.
BoundaryMap classify_boundaries(const Grid3D& mask);

// Throws BoundaryError for: non-finite or negative U_in, inconsistent map extents, or no Outlet
// face (fully solid domain or blocked outlet: the pressure would have no Dirichlet reference and
// inflow could not leave).
void validate_boundaries(const BoundaryMap& map, const BoundaryConditions& bc);

// Writes boundary-controlled faces only; Interior faces are never modified. Idempotent.
//   Inlet -> U_in;  SlipWall, SolidWall, Inactive -> 0;  Outlet u(Nx,j,k) -> u(Nx-1,j,k)
// (after the prescribed faces are set, so a one-cell-long tunnel copies the inlet value).
// Validates first; throws BoundaryError if the map does not match the grid.
void apply_velocity_bcs(MacGrid& mac, const BoundaryMap& map, const BoundaryConditions& bc);

// Writes only the prescribed faces (Inlet, SlipWall, SolidWall, Inactive); Outlet and Interior
// faces are untouched. Used after a pressure projection, whose outlet velocities are final and
// must not be overwritten by the zero-gradient predictor copy.
void apply_prescribed_velocity_bcs(MacGrid& mac, const BoundaryMap& map, const BoundaryConditions& bc);

// --- tangential ghost rules -------------------------------------------------------------------

enum class VelocityComponent { U, V, W };

// Value of the tangential neighbor of the velocity sample (component, i, j, k) one step along
// `axis` (0=x, 1=y, 2=z) in direction `side` (-1 or +1). `axis` must differ from the
// component's own normal axis. Returns the stored neighbor when it is a fluid-side sample;
// otherwise a ghost value by reflection about the boundary halfway between the samples:
//   crossing y/z tunnel walls (free-slip)      ghost = +value   (zero normal derivative)
//   crossing x = Lx (outlet, zero gradient)    ghost = +value
//   crossing x = 0 (inlet, zero transverse)    ghost = -value   (average 0 on the inlet plane)
//   neighbor Inactive (inside an obstacle)     ghost = -value   (no-slip, u = 0 on the voxel face)
// A SolidWall neighbor (on an obstacle's face) is returned as stored (0 after apply_velocity_bcs):
// at stair-step corners the wall is then taken at that sample, not halfway.
// Throws std::invalid_argument for a normal-direction axis, a bad side, or an Inactive sample.
double tangential_neighbor(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component,
                           std::size_t i, std::size_t j, std::size_t k, int axis, int side);

// --- pressure semantics for the (future) projection ----------------------------------------

enum class PressureFaceBC : std::uint8_t {
  Coupled,    // Interior face: couples two fluid-cell pressures; velocity is corrected
  Neumann,    // prescribed-velocity face: excluded from the Laplacian; velocity never corrected
  Dirichlet,  // Outlet face: p = 0 on the face plane; velocity corrected with the ghost pressure
};

[[nodiscard]] constexpr PressureFaceBC pressure_condition(VelocityFaceBC bc) noexcept {
  if (bc == VelocityFaceBC::Interior) return PressureFaceBC::Coupled;
  if (bc == VelocityFaceBC::Outlet) return PressureFaceBC::Dirichlet;
  return PressureFaceBC::Neumann;
}

// Ghost pressure beyond an Outlet face so that the face value (p_ghost + p_cell)/2 = 0.
[[nodiscard]] constexpr double outlet_ghost_pressure(double p_cell) noexcept { return -p_cell; }

// --- diagnostics ----------------------------------------------------------------------------

struct BoundaryFluxes {
  double inlet = 0.0;       // m^3/s entering through Inlet faces (+x)
  double outlet = 0.0;      // m^3/s leaving through Outlet faces (+x)
  double wall_leak = 0.0;   // sum |normal velocity| * area over SlipWall/SolidWall/Inactive faces
  double inlet_area = 0.0;  // m^2 of open inlet
  double outlet_area = 0.0; // m^2 of open outlet
};

BoundaryFluxes boundary_fluxes(const MacGrid& mac, const BoundaryMap& map);

}  // namespace aeroflow
