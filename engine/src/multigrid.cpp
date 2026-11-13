#include "aeroflow/multigrid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace aeroflow {

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint8_t kFace[6] = {1, 2, 4, 8, 16, 32};  // -x +x -y +y -z +z (PressureSystem bits)
constexpr std::uint32_t kUnset = std::numeric_limits<std::uint32_t>::max();

// Linear index of the face below cell coordinate c[axis] (c[axis] in 0..n[axis]) along `axis`.
std::size_t face_index(const std::array<std::size_t, 3>& n, int axis, std::size_t i, std::size_t j, std::size_t k) {
  if (axis == 0) return i + (n[0] + 1) * (j + n[1] * k);
  if (axis == 1) return i + n[0] * (j + (n[1] + 1) * k);
  return i + n[0] * (j + n[1] * k);
}

std::size_t face_count(const std::array<std::size_t, 3>& n, int axis) {
  return (n[0] + (axis == 0)) * (n[1] + (axis == 1)) * (n[2] + (axis == 2));
}

template <typename T>
std::size_t bytes(const std::vector<T>& v) {
  return v.capacity() * sizeof(T);
}

}  // namespace

void validate_multigrid_settings(const MultigridSettings& s) {
  if (s.pre_sweeps == 0 || s.post_sweeps == 0 || s.coarse_sweeps == 0) {
    throw std::invalid_argument("multigrid sweep counts must be at least 1");
  }
  if (s.pre_sweeps != s.post_sweeps) {
    throw std::invalid_argument("multigrid pre_sweeps and post_sweeps must be equal (symmetric cycle)");
  }
  if (s.max_levels == 0) throw std::invalid_argument("multigrid max_levels must be at least 1");
  if (!(s.semi_coarsening_ratio >= 1.0) || !std::isfinite(s.semi_coarsening_ratio)) {
    throw std::invalid_argument("multigrid semi_coarsening_ratio must be a finite number >= 1");
  }
}

// --- construction -------------------------------------------------------------------------------

Multigrid::Multigrid(const Grid3D& mask, const BoundaryMap& map, const PressureSystem& system, MultigridSettings settings)
    : map_(map), system_(system), settings_(settings) {
  const auto dims = system.dims();
  if (dims[0] != mask.nx() || dims[1] != mask.ny() || dims[2] != mask.nz() || &map != &system.boundary_map()) {
    throw std::invalid_argument("multigrid: mask, boundary map and pressure system describe different grids");
  }
  build(dims, {mask.dx(), mask.dy(), mask.dz()});
}

Multigrid::Multigrid(const PressureSystem& system, MultigridSettings settings)
    : map_(system.boundary_map()), system_(system), settings_(settings) {
  build(system.dims(), system.spacing());
}

void Multigrid::build(std::array<std::size_t, 3> dims, std::array<double, 3> spacing) {
  const auto start = Clock::now();
  validate_multigrid_settings(settings_);
  if (map_.nx != dims[0] || map_.ny != dims[1] || map_.nz != dims[2]) {
    throw std::invalid_argument("multigrid: boundary map does not match the pressure system");
  }
  build_level0(dims, spacing);
  finish_level(0);
  while (levels_.size() < settings_.max_levels && unknowns(levels_.size() - 1) > settings_.coarsest_unknowns) {
    const std::size_t fine = levels_.size() - 1;
    const auto& n = levels_[fine].n;
    if (n[0] < 2 && n[1] < 2 && n[2] < 2) break;
    build_coarse(fine);
    build_transfer(fine);
    finish_level(fine + 1);
    build_transfer_masks(fine);
  }
  // Open face areas are only needed to aggregate the next level; release them and re-account.
  for (std::size_t l = 1; l < levels_.size(); ++l) {
    for (auto& a : levels_[l].area) {
      info_[l].memory_bytes -= bytes(a);
      std::vector<double>().swap(a);
    }
  }
  setup_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

void Multigrid::build_level0(std::array<std::size_t, 3> dims, std::array<double, 3> h) {
  Level L;
  L.n = dims;
  for (int a = 0; a < 3; ++a) {
    const auto ua = static_cast<std::size_t>(a);
    L.width[ua].assign(L.n[ua], h[ua]);
    L.center[ua].resize(L.n[ua]);
    for (std::size_t i = 0; i < L.n[ua]; ++i) L.center[ua][i] = (static_cast<double>(i) + 0.5) * h[ua];
  }
  v0_ = h[0] * h[1] * h[2];
  levels_.push_back(std::move(L));
}

void Multigrid::build_coarse(std::size_t fine_index) {
  const Level& F = levels_[fine_index];
  Level C;
  // Semi-coarsening: coarsen the axes whose (full-cell) width is within the ratio of the smallest.
  double hmin = std::numeric_limits<double>::infinity();
  for (std::size_t a = 0; a < 3; ++a)
    if (F.n[a] >= 2) hmin = std::min(hmin, F.width[a][0]);
  for (std::size_t a = 0; a < 3; ++a) {
    C.coarsened[a] = F.n[a] >= 2 && F.width[a][0] <= settings_.semi_coarsening_ratio * hmin * (1.0 + 1e-12);
    C.n[a] = C.coarsened[a] ? (F.n[a] + 1) / 2 : F.n[a];
    C.width[a].resize(C.n[a]);
    C.center[a].resize(C.n[a]);
    double start = 0.0;
    for (std::size_t I = 0; I < C.n[a]; ++I) {
      double w = 0.0;
      if (C.coarsened[a]) {
        w = F.width[a][2 * I] + (2 * I + 1 < F.n[a] ? F.width[a][2 * I + 1] : 0.0);
      } else {
        w = F.width[a][I];
      }
      C.width[a][I] = w;
      C.center[a][I] = start + 0.5 * w;
      start += w;
    }
  }
  auto parent_of = [&](std::size_t a, std::size_t i) { return C.coarsened[a] ? i / 2 : i; };

  // Fluid coarse cells: any fluid child.
  const std::size_t ncells = C.n[0] * C.n[1] * C.n[2];
  std::vector<std::uint8_t> fluid(ncells, 0);
  const std::size_t nfine = unknowns(fine_index);
  for (std::size_t u = 0; u < nfine; ++u) {
    const std::size_t c = fine_index == 0 ? system_.cell_of_unknown(u) : F.cell_of_unknown[u];
    const std::size_t i = c % F.n[0], j = (c / F.n[0]) % F.n[1], k = c / (F.n[0] * F.n[1]);
    fluid[parent_of(0, i) + C.n[0] * (parent_of(1, j) + C.n[1] * parent_of(2, k))] = 1;
  }
  C.unknown_of_cell.assign(ncells, -1);
  for (std::size_t c = 0; c < ncells; ++c) {
    if (!fluid[c]) continue;
    C.unknown_of_cell[c] = static_cast<std::int32_t>(C.cell_of_unknown.size());
    C.cell_of_unknown.push_back(static_cast<std::uint32_t>(c));
  }

  // Face-opening aggregation: a coarse face's open area is the sum of the open areas of the finer
  // faces that lie in its plane (finer faces inside a coarse cell are internal and dropped).
  for (int a = 0; a < 3; ++a) C.area[static_cast<std::size_t>(a)].assign(face_count(C.n, a), 0.0);
  const std::array<double, 3> h0 = {levels_[0].width[0][0], levels_[0].width[1][0], levels_[0].width[2][0]};
  for (int a = 0; a < 3; ++a) {
    const auto ua = static_cast<std::size_t>(a);
    std::array<std::size_t, 3> fn = F.n;
    fn[ua] += 1;  // face counts along `a`
    for (std::size_t k = 0; k < fn[2]; ++k)
      for (std::size_t j = 0; j < fn[1]; ++j)
        for (std::size_t i = 0; i < fn[0]; ++i) {
          double area = 0.0;
          if (fine_index == 0) {
            VelocityFaceBC bc;
            if (a == 0) bc = map_.u[face_index(F.n, 0, i, j, k)];
            else if (a == 1) bc = map_.v[face_index(F.n, 1, i, j, k)];
            else bc = map_.w[face_index(F.n, 2, i, j, k)];
            // Interior faces couple two fluid cells; Outlet faces carry the Dirichlet condition.
            if (bc == VelocityFaceBC::Interior || bc == VelocityFaceBC::Outlet) {
              area = a == 0 ? h0[1] * h0[2] : a == 1 ? h0[0] * h0[2] : h0[0] * h0[1];
            }
          } else {
            area = F.area[ua][face_index(F.n, a, i, j, k)];
          }
          if (area == 0.0) continue;
          std::array<std::size_t, 3> idx = {i, j, k};
          std::array<std::size_t, 3> cidx{};
          bool internal = false;
          for (std::size_t b = 0; b < 3; ++b) {
            if (b == ua) {
              if (!C.coarsened[b]) {
                cidx[b] = idx[b];
              } else if (idx[b] == F.n[b]) {
                cidx[b] = C.n[b];
              } else if (idx[b] % 2 == 1) {
                internal = true;
              } else {
                cidx[b] = idx[b] / 2;
              }
            } else {
              cidx[b] = parent_of(b, idx[b]);
            }
          }
          if (internal) continue;
          C.area[ua][face_index(C.n, a, cidx[0], cidx[1], cidx[2])] += area;
        }
  }

  // Conductances: open area / centre-to-centre distance (true centres, so partial cells are
  // exact); outlet faces: open area / half width (Dirichlet on the face plane).
  for (int a = 0; a < 3; ++a) {
    const auto ua = static_cast<std::size_t>(a);
    C.cond[ua].assign(C.area[ua].size(), 0.0);
    std::array<std::size_t, 3> fn = C.n;
    fn[ua] += 1;
    for (std::size_t k = 0; k < fn[2]; ++k)
      for (std::size_t j = 0; j < fn[1]; ++j)
        for (std::size_t i = 0; i < fn[0]; ++i) {
          const std::size_t f = face_index(C.n, a, i, j, k);
          const double area = C.area[ua][f];
          if (area == 0.0) continue;
          const std::array<std::size_t, 3> idx = {i, j, k};
          const std::size_t I = idx[ua];
          if (I == 0) {
            throw std::logic_error("multigrid: open area on a low domain face");  // inlet/walls are closed
          }
          if (I == C.n[ua]) {
            if (a != 0) throw std::logic_error("multigrid: open area on a side or top domain face");
            C.cond[ua][f] = area / (0.5 * C.width[ua][I - 1]);  // outlet
          } else {
            C.cond[ua][f] = area / (C.center[ua][I] - C.center[ua][I - 1]);
          }
        }
  }
  C.diag.assign(C.cell_of_unknown.size(), 0.0);
  for (std::size_t u = 0; u < C.cell_of_unknown.size(); ++u) {
    const std::size_t c = C.cell_of_unknown[u];
    const std::size_t i = c % C.n[0], j = (c / C.n[0]) % C.n[1], k = c / (C.n[0] * C.n[1]);
    C.diag[u] = C.cond[0][face_index(C.n, 0, i, j, k)] + C.cond[0][face_index(C.n, 0, i + 1, j, k)] +
                C.cond[1][face_index(C.n, 1, i, j, k)] + C.cond[1][face_index(C.n, 1, i, j + 1, k)] +
                C.cond[2][face_index(C.n, 2, i, j, k)] + C.cond[2][face_index(C.n, 2, i, j, k + 1)];
  }
  levels_.push_back(std::move(C));
}

void Multigrid::finish_level(std::size_t index) {
  Level& L = levels_[index];
  const std::size_t n = unknowns(index);
  auto cell = [&](std::size_t u) -> std::size_t {
    return index == 0 ? system_.cell_of_unknown(u) : L.cell_of_unknown[u];
  };
  L.red.clear();
  L.black.clear();
  for (std::size_t u = 0; u < n; ++u) {
    const std::size_t c = cell(u);
    const std::size_t i = c % L.n[0], j = (c / L.n[0]) % L.n[1], k = c / (L.n[0] * L.n[1]);
    ((i + j + k) % 2 == 0 ? L.red : L.black).push_back(static_cast<std::uint32_t>(u));
  }
  L.red.shrink_to_fit();
  L.black.shrink_to_fit();

  MultigridLevelInfo info;
  info.dims = L.n;
  info.coarsened = L.coarsened;
  info.unknowns = n;
  if (index == 0) {
    info.components = system_.components().size();
    for (const auto& comp : system_.components()) info.enclosed_components += comp.has_outlet ? 0 : 1;
    for (std::size_t u = 0; u < n; ++u) info.isolated_cells += system_.diagonal(u) == 0.0 ? 1 : 0;
  } else {
    // Components through open coarse faces (they may differ from the fine components).
    L.component.assign(n, kUnset);
    std::vector<std::uint32_t> stack;
    for (std::size_t seed = 0; seed < n; ++seed) {
      if (L.component[seed] != kUnset) continue;
      const auto id = static_cast<std::uint32_t>(L.component_has_outlet.size());
      std::uint8_t outlet = 0;
      L.component[seed] = id;
      stack.push_back(static_cast<std::uint32_t>(seed));
      while (!stack.empty()) {
        const std::uint32_t u = stack.back();
        stack.pop_back();
        const std::size_t c = L.cell_of_unknown[u];
        const std::size_t i = c % L.n[0], j = (c / L.n[0]) % L.n[1], k = c / (L.n[0] * L.n[1]);
        if (i + 1 == L.n[0] && L.cond[0][face_index(L.n, 0, L.n[0], j, k)] > 0.0) outlet = 1;
        const std::array<std::size_t, 3> idx = {i, j, k};
        for (int a = 0; a < 3; ++a) {
          const auto ua = static_cast<std::size_t>(a);
          for (int side = 0; side < 2; ++side) {
            std::array<std::size_t, 3> f = idx;
            if (side == 1) f[ua] += 1;
            if (f[ua] == 0 || f[ua] == L.n[ua]) continue;  // domain boundary
            if (L.cond[ua][face_index(L.n, a, f[0], f[1], f[2])] == 0.0) continue;
            std::array<std::size_t, 3> nb = idx;
            nb[ua] = side == 1 ? idx[ua] + 1 : idx[ua] - 1;
            const auto v = static_cast<std::uint32_t>(L.unknown_of_cell[nb[0] + L.n[0] * (nb[1] + L.n[1] * nb[2])]);
            if (L.component[v] == kUnset) {
              L.component[v] = id;
              stack.push_back(v);
            }
          }
        }
      }
      L.component_has_outlet.push_back(outlet);
    }
    info.components = L.component_has_outlet.size();
    for (const auto o : L.component_has_outlet) info.enclosed_components += o ? 0 : 1;
    for (const double d : L.diag) info.isolated_cells += d == 0.0 ? 1 : 0;
  }

  // Work vectors (level 0 borrows x and f from the caller).
  L.r.assign(n, 0.0);
  if (index > 0) {
    L.x.assign(n, 0.0);
    L.f.assign(n, 0.0);
  }
  std::size_t mem = bytes(L.unknown_of_cell) + bytes(L.cell_of_unknown) + bytes(L.diag) + bytes(L.red) +
                    bytes(L.black) + bytes(L.component) + bytes(L.component_has_outlet) + bytes(L.x) + bytes(L.f) +
                    bytes(L.r);
  for (std::size_t a = 0; a < 3; ++a) {
    mem += bytes(L.width[a]) + bytes(L.center[a]) + bytes(L.cond[a]) + bytes(L.area[a]);
  }
  info.memory_bytes = mem;
  info_.push_back(info);
}

void Multigrid::build_transfer(std::size_t fine_index) {
  Level& F = levels_[fine_index];
  const Level& C = levels_[fine_index + 1];
  for (std::size_t a = 0; a < 3; ++a) {
    F.parent[a].resize(F.n[a]);
    F.step[a].assign(F.n[a], 0);
    F.weight[a].assign(F.n[a], 0.0);
    for (std::size_t i = 0; i < F.n[a]; ++i) {
      const std::size_t I = C.coarsened[a] ? i / 2 : i;
      F.parent[a][i] = static_cast<std::uint32_t>(I);
      if (!C.coarsened[a] || settings_.transfer == MultigridTransfer::Constant) continue;
      const double d = F.center[a][i] - C.center[a][I];
      if (std::abs(d) <= 1e-12 * C.width[a][I]) continue;  // one-child cell: same centre
      const bool up = d > 0.0;
      if (up ? I + 1 >= C.n[a] : I == 0) continue;  // domain boundary: constant extrapolation
      const std::size_t J = up ? I + 1 : I - 1;
      F.step[a][i] = static_cast<std::int8_t>(up ? 1 : -1);
      F.weight[a][i] = std::abs(d) / std::abs(C.center[a][J] - C.center[a][I]);
    }
  }
  // Account the transfer arrays in the fine level's memory.
  std::size_t mem = 0;
  for (std::size_t a = 0; a < 3; ++a) mem += bytes(F.parent[a]) + bytes(F.step[a]) + bytes(F.weight[a]);
  info_[fine_index].memory_bytes += mem;
}

std::size_t Multigrid::memory_bytes() const noexcept {
  std::size_t total = 0;
  for (const auto& i : info_) total += i.memory_bytes;
  return total;
}

std::int32_t Multigrid::unknown_of_cell(std::size_t level, std::size_t i, std::size_t j, std::size_t k) const {
  const Level& L = levels_.at(level);
  const std::size_t c = i + L.n[0] * (j + L.n[1] * k);
  return level == 0 ? system_.unknown_of_cell(c) : L.unknown_of_cell[c];
}

double Multigrid::conductance(std::size_t level, int axis, std::size_t i, std::size_t j, std::size_t k) const {
  const Level& L = levels_.at(level);
  if (level == 0) throw std::invalid_argument("multigrid: level 0 uses the PressureSystem coefficients");
  return L.cond[static_cast<std::size_t>(axis)][face_index(L.n, axis, i, j, k)];
}

// --- transfers ----------------------------------------------------------------------------------

Multigrid::Stencil Multigrid::stencil(std::size_t fine_index, std::size_t u) const {
  const Level& F = levels_[fine_index];
  const Level& C = levels_[fine_index + 1];
  const std::size_t c = fine_index == 0 ? system_.cell_of_unknown(u) : F.cell_of_unknown[u];
  const std::array<std::size_t, 3> idx = {c % F.n[0], (c / F.n[0]) % F.n[1], c / (F.n[0] * F.n[1])};
  std::array<std::size_t, 3> P{};
  std::array<int, 3> s{};
  std::array<double, 3> w{};
  for (std::size_t a = 0; a < 3; ++a) {
    P[a] = F.parent[a][idx[a]];
    s[a] = F.step[a][idx[a]];
    w[a] = F.weight[a][idx[a]];
  }
  // Interpolate along an axis only through an open coarse face.
  for (int a = 0; a < 3; ++a) {
    const auto ua = static_cast<std::size_t>(a);
    if (s[ua] == 0) continue;
    std::array<std::size_t, 3> f = P;
    if (s[ua] > 0) f[ua] += 1;
    if (C.cond[ua][face_index(C.n, a, f[0], f[1], f[2])] == 0.0) {
      s[ua] = 0;
      w[ua] = 0.0;
    }
  }
  const std::size_t pc = P[0] + C.n[0] * (P[1] + C.n[1] * P[2]);
  const std::int32_t parent = C.unknown_of_cell[pc];
  const std::uint32_t comp = C.component[static_cast<std::size_t>(parent)];
  Stencil st;
  st.unknown[0] = static_cast<std::uint32_t>(parent);
  st.weight[0] = 0.0;
  st.size = 1;
  for (int combo = 0; combo < 8; ++combo) {
    double weight = 1.0;
    std::array<std::size_t, 3> q = P;
    for (int a = 0; a < 3; ++a) {
      const auto ua = static_cast<std::size_t>(a);
      const bool shifted = (combo >> a) & 1;
      if (shifted) {
        if (s[ua] == 0) {
          weight = 0.0;
          break;
        }
        weight *= w[ua];
        q[ua] = s[ua] > 0 ? P[ua] + 1 : P[ua] - 1;
      } else {
        weight *= 1.0 - w[ua];
      }
    }
    if (weight == 0.0) continue;
    if (combo == 0) {
      st.weight[0] += weight;
      continue;
    }
    const std::int32_t v = C.unknown_of_cell[q[0] + C.n[0] * (q[1] + C.n[1] * q[2])];
    if (v < 0 || C.component[static_cast<std::size_t>(v)] != comp) {
      st.weight[0] += weight;  // solid or another region: the parent takes the weight
      continue;
    }
    st.unknown[static_cast<std::size_t>(st.size)] = static_cast<std::uint32_t>(v);
    st.weight[static_cast<std::size_t>(st.size)] = weight;
    ++st.size;
  }
  return st;
}

void Multigrid::build_transfer_masks(std::size_t fine_index) {
  Level& F = levels_[fine_index];
  const std::size_t n = unknowns(fine_index);
  F.transfer_mask.assign(n, 0);
  if (settings_.transfer == MultigridTransfer::Constant) {  // all axes off: P picks the parent
    info_[fine_index].memory_bytes += bytes(F.transfer_mask);
    return;
  }
  for (std::size_t u = 0; u < n; ++u) {
    // Same rules as stencil(): an axis interpolates only through an open coarse face; a stencil
    // cell contributes its own weight only if it is fluid and in the parent's coarse region.
    const Stencil st = stencil(fine_index, u);
    const std::size_t c = fine_index == 0 ? system_.cell_of_unknown(u) : F.cell_of_unknown[u];
    const std::array<std::size_t, 3> idx = {c % F.n[0], (c / F.n[0]) % F.n[1], c / (F.n[0] * F.n[1])};
    const Level& C = levels_[fine_index + 1];
    std::uint16_t mask = 0;
    std::array<std::size_t, 3> P{};
    for (std::size_t a = 0; a < 3; ++a) P[a] = F.parent[a][idx[a]];
    for (std::size_t a = 0; a < 3; ++a) {
      const int s = F.step[a][idx[a]];
      if (s == 0) continue;
      std::array<std::size_t, 3> f = P;
      if (s > 0) f[a] += 1;
      if (C.cond[a][face_index(C.n, static_cast<int>(a), f[0], f[1], f[2])] != 0.0) {
        mask = static_cast<std::uint16_t>(mask | (1u << (8 + a)));
      }
    }
    // Mark the combos that stencil() kept as separate entries.
    for (int m = 1; m < st.size; ++m) {
      const std::size_t cc = C.cell_of_unknown[st.unknown[static_cast<std::size_t>(m)]];
      const std::array<std::size_t, 3> q = {cc % C.n[0], (cc / C.n[0]) % C.n[1], cc / (C.n[0] * C.n[1])};
      int combo = 0;
      for (std::size_t a = 0; a < 3; ++a)
        if (q[a] != P[a]) combo |= 1 << a;
      mask = static_cast<std::uint16_t>(mask | (1u << combo));
    }
    F.transfer_mask[u] = mask;
  }
  info_[fine_index].memory_bytes += bytes(F.transfer_mask);
}

// Visits every (fine unknown, coarse unknown, weight) entry of P in fine-box order. Prolongation and
// restriction both use it, so R = P^T holds by construction.
template <typename Emit>
void Multigrid::for_each_transfer(std::size_t fine_index, Emit&& emit) const {
  const Level& F = levels_[fine_index];
  const Level& C = levels_[fine_index + 1];
  const std::ptrdiff_t cx = 1, cy = static_cast<std::ptrdiff_t>(C.n[0]),
                       cz = static_cast<std::ptrdiff_t>(C.n[0] * C.n[1]);
  std::size_t cell = 0;
  for (std::size_t k = 0; k < F.n[2]; ++k) {
    const std::size_t K = F.parent[2][k];
    const int sz = F.step[2][k];
    const double wz0 = F.weight[2][k];
    for (std::size_t j = 0; j < F.n[1]; ++j) {
      const std::size_t J = F.parent[1][j];
      const int sy = F.step[1][j];
      const double wy0 = F.weight[1][j];
      const std::size_t row = C.n[0] * (J + C.n[1] * K);
      for (std::size_t i = 0; i < F.n[0]; ++i, ++cell) {
        const std::int32_t uf = fine_index == 0 ? system_.unknown_of_cell(cell) : F.unknown_of_cell[cell];
        if (uf < 0) continue;
        const auto u = static_cast<std::size_t>(uf);
        const std::uint16_t m = F.transfer_mask[u];
        const std::size_t pc = F.parent[0][i] + row;
        const auto parent = static_cast<std::size_t>(C.unknown_of_cell[pc]);
        const double wx = (m & 0x100) ? F.weight[0][i] : 0.0;
        const double wy = (m & 0x200) ? wy0 : 0.0;
        const double wz = (m & 0x400) ? wz0 : 0.0;
        if (wx == 0.0 && wy == 0.0 && wz == 0.0) {
          emit(u, parent, 1.0);
          continue;
        }
        const std::ptrdiff_t ox = F.step[0][i] * cx, oy = sy * cy, oz = sz * cz;
        double w_parent = 0.0;
        for (int combo = 0; combo < 8; ++combo) {
          const double w = ((combo & 1) ? wx : 1.0 - wx) * ((combo & 2) ? wy : 1.0 - wy) * ((combo & 4) ? wz : 1.0 - wz);
          if (w == 0.0) continue;
          if (combo == 0 || !(m & (1u << combo))) {
            w_parent += w;  // the parent itself, or a solid / other-region cell giving its weight back
            continue;
          }
          const std::ptrdiff_t off = ((combo & 1) ? ox : 0) + ((combo & 2) ? oy : 0) + ((combo & 4) ? oz : 0);
          emit(u, static_cast<std::size_t>(C.unknown_of_cell[static_cast<std::size_t>(static_cast<std::ptrdiff_t>(pc) + off)]), w);
        }
        emit(u, parent, w_parent);
      }
    }
  }
}

void Multigrid::prolong(std::size_t level, std::span<const double> coarse, std::span<double> fine) const {
  if (level + 1 >= levels_.size()) throw std::invalid_argument("multigrid: no coarser level");
  std::fill(fine.begin(), fine.end(), 0.0);
  prolong_add(level, coarse, fine);
}

void Multigrid::prolong_add(std::size_t level, std::span<const double> coarse, std::span<double> fine) const {
  for_each_transfer(level, [&](std::size_t u, std::size_t c, double w) { fine[u] += w * coarse[c]; });
}

void Multigrid::restrict_transpose(std::size_t level, std::span<const double> fine, std::span<double> coarse) const {
  if (level + 1 >= levels_.size()) throw std::invalid_argument("multigrid: no coarser level");
  std::fill(coarse.begin(), coarse.end(), 0.0);
  for_each_transfer(level, [&](std::size_t u, std::size_t c, double w) { coarse[c] += w * fine[u]; });
}

void Multigrid::prolong_reference(std::size_t level, std::span<const double> coarse, std::span<double> fine) const {
  for (std::size_t u = 0; u < fine.size(); ++u) {
    const Stencil st = stencil(level, u);
    double v = 0.0;
    for (int n = 0; n < st.size; ++n) v += st.weight[static_cast<std::size_t>(n)] * coarse[st.unknown[static_cast<std::size_t>(n)]];
    fine[u] = v;
  }
}

// --- level operators and smoothing --------------------------------------------------------------

double Multigrid::off_diagonal_sum(std::size_t level, std::uint32_t u, std::span<const double> x) const {
  if (level == 0) {
    // Same terms in the same order as PressureSystem::apply.
    const auto c = static_cast<std::ptrdiff_t>(system_.cell_of_unknown(u));
    const auto& n = levels_[0].n;
    const std::ptrdiff_t sy = static_cast<std::ptrdiff_t>(n[0]), sz = static_cast<std::ptrdiff_t>(n[0] * n[1]);
    const auto h2 = system_.inverse_spacing_squared();
    const std::uint8_t f = system_.coupled_faces(u);
    auto at = [&](std::ptrdiff_t cell) { return x[static_cast<std::size_t>(system_.unknown_of_cell(static_cast<std::size_t>(cell)))]; };
    double off = 0.0;
    // Unknowns are numbered x-fastest over fluid cells, so a coupled -x/+x neighbour is u -/+ 1.
    if (f & kFace[0]) off += h2[0] * x[u - 1];
    if (f & kFace[1]) off += h2[0] * x[u + 1];
    if (f & kFace[2]) off += h2[1] * at(c - sy);
    if (f & kFace[3]) off += h2[1] * at(c + sy);
    if (f & kFace[4]) off += h2[2] * at(c - sz);
    if (f & kFace[5]) off += h2[2] * at(c + sz);
    return off;
  }
  const Level& L = levels_[level];
  const std::size_t c = L.cell_of_unknown[u];
  const std::size_t i = c % L.n[0], j = (c / L.n[0]) % L.n[1], k = c / (L.n[0] * L.n[1]);
  const std::size_t sy = L.n[0], sz = L.n[0] * L.n[1];
  auto at = [&](std::size_t cell) { return x[static_cast<std::size_t>(L.unknown_of_cell[cell])]; };
  double off = 0.0;
  double t = 0.0;
  if (i > 0 && (t = L.cond[0][face_index(L.n, 0, i, j, k)]) != 0.0) off += t * x[u - 1];
  if (i + 1 < L.n[0] && (t = L.cond[0][face_index(L.n, 0, i + 1, j, k)]) != 0.0) off += t * x[u + 1];
  if (j > 0 && (t = L.cond[1][face_index(L.n, 1, i, j, k)]) != 0.0) off += t * at(c - sy);
  if (j + 1 < L.n[1] && (t = L.cond[1][face_index(L.n, 1, i, j + 1, k)]) != 0.0) off += t * at(c + sy);
  if (k > 0 && (t = L.cond[2][face_index(L.n, 2, i, j, k)]) != 0.0) off += t * at(c - sz);
  if (k + 1 < L.n[2] && (t = L.cond[2][face_index(L.n, 2, i, j, k + 1)]) != 0.0) off += t * at(c + sz);
  return off;
}

double Multigrid::diagonal(std::size_t level, std::uint32_t u) const {
  return level == 0 ? system_.diagonal(u) : levels_[level].diag[u];
}

void Multigrid::apply_operator(std::size_t level, std::span<const double> x, std::span<double> y) const {
  const std::size_t n = unknowns(level);
  if (x.size() != n || y.size() != n) throw std::invalid_argument("multigrid: vector size does not match the level");
  for (std::size_t u = 0; u < n; ++u) {
    const auto uu = static_cast<std::uint32_t>(u);
    y[u] = diagonal(level, uu) * x[u] - off_diagonal_sum(level, uu, x);
  }
}

// Inside the cycle a zero row (isolated cell) is treated as having a unit diagonal, so every
// level operator used by the smoother and the residual is positive on the diagonal.
void Multigrid::sweep(std::size_t level, const std::vector<std::uint32_t>& color, std::span<const double> f,
                      std::span<double> x) const {
  for (const std::uint32_t u : color) {
    const double d = diagonal(level, u);
    x[u] = d > 0.0 ? (f[u] + off_diagonal_sum(level, u, x)) / d : f[u];
  }
}

void Multigrid::smooth(std::size_t level, std::size_t sweeps, bool reversed, std::span<const double> f,
                       std::span<double> x) const {
  const Level& L = levels_[level];
  for (std::size_t s = 0; s < sweeps; ++s) {
    if (!reversed) {
      sweep(level, L.red, f, x);
      sweep(level, L.black, f, x);
    } else {
      sweep(level, L.black, f, x);
      sweep(level, L.red, f, x);
    }
  }
}

void Multigrid::residual(std::size_t level, std::span<const double> f, std::span<const double> x,
                         std::span<double> r) const {
  const std::size_t n = unknowns(level);
  for (std::size_t u = 0; u < n; ++u) {
    const auto uu = static_cast<std::uint32_t>(u);
    const double d = diagonal(level, uu);
    r[u] = f[u] - ((d > 0.0 ? d : 1.0) * x[u] - off_diagonal_sum(level, uu, x));
  }
}

// --- V-cycle ------------------------------------------------------------------------------------

void Multigrid::apply(std::span<const double> r, std::span<double> z) {
  const std::size_t n0 = unknowns(0);
  if (r.size() != n0 || z.size() != n0) throw std::invalid_argument("multigrid: vector size does not match the grid");
  std::fill(z.begin(), z.end(), 0.0);
  const std::size_t L = levels_.size();
  if (L == 1) {
    smooth(0, settings_.coarse_sweeps, false, r, z);
    smooth(0, settings_.coarse_sweeps, true, r, z);
    return;
  }
  auto x_of = [&](std::size_t l) -> std::span<double> { return l == 0 ? z : std::span<double>(levels_[l].x); };
  auto f_of = [&](std::size_t l) -> std::span<const double> { return l == 0 ? r : std::span<const double>(levels_[l].f); };
  // Optional stage timing (benchmarks only).
  auto tick = Clock::now();
  auto lap = [&](double& bucket) {
    if (!profiling_) return;
    const auto now = Clock::now();
    bucket += std::chrono::duration<double, std::milli>(now - tick).count();
    tick = now;
  };
  if (profiling_) ++profile_.cycles;
  auto& P = profile_;

  // Down: pre-smooth, restrict the residual (level 0 -> 1 carries K_0 = V_0 A).
  for (std::size_t l = 0; l + 1 < L; ++l) {
    auto x = x_of(l);
    if (l > 0) std::fill(x.begin(), x.end(), 0.0);
    smooth(l, settings_.pre_sweeps, false, f_of(l), x);
    lap(l == 0 ? P.fine_smooth : P.coarse_levels);
    residual(l, f_of(l), x, levels_[l].r);
    lap(l == 0 ? P.fine_residual : P.coarse_levels);
    auto& fc = levels_[l + 1].f;
    restrict_transpose(l, levels_[l].r, fc);
    if (l == 0)
      for (double& v : fc) v *= v0_;
    lap(l == 0 ? P.fine_restrict : P.coarse_levels);
  }
  // Coarsest: fixed forward then reversed sweeps from zero.
  {
    auto& x = levels_[L - 1].x;
    std::fill(x.begin(), x.end(), 0.0);
    smooth(L - 1, settings_.coarse_sweeps, false, levels_[L - 1].f, x);
    smooth(L - 1, settings_.coarse_sweeps, true, levels_[L - 1].f, x);
    lap(P.coarse_levels);
  }
  // Up: prolongate and add the correction, post-smooth in reversed order.
  for (std::size_t l = L - 1; l-- > 0;) {
    auto x = x_of(l);
    prolong_add(l, levels_[l + 1].x, x);
    lap(l == 0 ? P.fine_prolong : P.coarse_levels);
    smooth(l, settings_.post_sweeps, true, f_of(l), x);
    lap(l == 0 ? P.fine_smooth : P.coarse_levels);
  }
}

}  // namespace aeroflow
