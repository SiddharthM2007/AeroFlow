#pragma once

#include <cstddef>

#include "aeroflow/boundary.hpp"
#include "aeroflow/mac_grid.hpp"

namespace aeroflow {

// Explicit momentum terms on the staggered MAC grid (Milestone 6).
//
// Advection, ADVECTIVE (non-conservative) form with first-order upwinding. For a u-face:
//   (u.grad)u = u du/dx + vbar du/dy + wbar du/dz,
//   vbar = average of the 4 v-faces around the u-face (cells on both sides x their two y-faces),
//   q df/dx_b ~ q > 0 ? q (f - f_{-b}) / h_b : q (f_{+b} - f) / h_b.
// Exactly zero for a uniform field; monotone for a frozen velocity; not discretely
// momentum-conserving (a flux form would be, at the cost of edge interpolation).
//
// Diffusion: 7-point Laplacian sum_b (f_{+b} - 2 f + f_{-b}) / h_b^2.
//
// Both are evaluated on Interior faces only: prescribed faces are fixed by the boundary
// conditions and outlet faces take the zero-gradient predictor copy. Neighbors come from
// velocity_neighbor(): stored values inside the grid (prescribed faces hold their exact
// boundary values; blocked faces hold the stationary solid's 0), and the 3B ghost rules where a
// tangential stencil leaves the fluid.

// Boundary-aware neighbor of the sample (component, i, j, k) one step along `axis`, `side` = +-1.
// Tangential directions follow tangential_neighbor(): +value across free-slip walls and the
// outlet, -value across the inlet and into Inactive (obstacle interior) faces. Along the
// component's own axis the stored neighbor is used; leaving the domain there (outlet side)
// gives the zero-gradient ghost +value.
double velocity_neighbor(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component, std::size_t i,
                         std::size_t j, std::size_t k, int axis, int side);

// -(u.grad)u at one Interior sample. Throws std::invalid_argument for non-Interior samples.
double advection_term(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component, std::size_t i,
                      std::size_t j, std::size_t k);

// Laplacian (without nu) at one Interior sample. Throws std::invalid_argument for non-Interior samples.
double laplacian_term(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component, std::size_t i,
                      std::size_t j, std::size_t k);

struct MomentumTerms {
  bool advection = true;
  bool diffusion = true;
  double viscosity = 0.0;  // kinematic, m^2/s
};

// out = u + dt * (-(u.grad)u + nu lap u) on Interior faces; out = u elsewhere. `out_*` must have
// the extents of mac.u/v/w (they are reused between steps; no allocation here).
void explicit_momentum_update(const MacGrid& mac, const BoundaryMap& map, const MomentumTerms& terms, double dt,
                              StaggeredArray& out_u, StaggeredArray& out_v, StaggeredArray& out_w);

struct VelocityBounds {
  double max_u = 0.0, max_v = 0.0, max_w = 0.0;
  bool finite = true;
};
VelocityBounds velocity_bounds(const MacGrid& mac);

// Explicit forward-Euler limit for upwind advection + central diffusion (non-negative update
// coefficients): dt <= 1 / (max|u|/dx + max|v|/dy + max|w|/dz + 2 nu (1/dx^2 + 1/dy^2 + 1/dz^2)).
// Returns +infinity when the denominator is 0 (no motion, no viscosity).
double stability_limit(const MacGrid& mac, double viscosity);

}  // namespace aeroflow
