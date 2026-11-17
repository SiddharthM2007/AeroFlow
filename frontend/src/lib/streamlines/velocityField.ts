/**
 * Continuous velocity from the staggered MAC samples of one simulation frame (Milestone 8B).
 *
 * Each component is interpolated trilinearly on its own sample grid, at its true locations:
 *   u at (i dx, (j+1/2) dy, (k+1/2) dz), v at ((i+1/2) dx, j dy, (k+1/2) dz), w likewise.
 * This reproduces the stored face values exactly and adds no information beyond them.
 *
 * Near walls the interpolation is conservative:
 *   - Only valid samples are used (faces inside solids, "Inactive", are invalid), with the
 *     weights renormalized over the valid ones. The no-penetration zeros stored on solid-wall and
 *     domain-wall faces are valid samples, so the wall-normal component still goes to zero at the
 *     wall; no slip or wall model is invented for the tangential components.
 *   - Beyond the outermost samples of a component (half a cell at side walls) the nearest samples
 *     are used (constant extrapolation, matching the solver's slip-wall ghost values).
 * A point whose stencil has no valid sample, or a non-finite value, has no velocity.
 */
import type { ScalarFieldData } from "../fields/format";

export type Vec3 = [number, number, number];

export interface MacVelocity {
  n: Vec3; // cells per axis
  h: Vec3; // spacing (m)
  origin: Vec3;
  /** Components on their face grids: u (nx+1, ny, nz), v (nx, ny+1, nz), w (nx, ny, nz+1). */
  comp: [Float32Array, Float32Array, Float32Array];
  valid: [Uint8Array, Uint8Array, Uint8Array];
  /** 1 = solid cell (x-fastest over n). */
  solid: Uint8Array;
}

/** Grid geometry and solid mask only (enough for seeding and domain/solid tests). */
export type GridGeometry = Pick<MacVelocity, "n" | "h" | "origin" | "solid">;

const LOCATIONS = ["face_x", "face_y", "face_z"] as const;

/** Assembles the velocity from the exported sim_u / sim_v / sim_w frames and the cell mask. */
export function macVelocityFromFields(
  u: ScalarFieldData,
  v: ScalarFieldData,
  w: ScalarFieldData,
  solid: Uint8Array,
): MacVelocity {
  const parts = [u, v, w];
  const g = u.meta.grid;
  const n: Vec3 = [g.resolution.nx, g.resolution.ny, g.resolution.nz];
  parts.forEach((p, a) => {
    if (p.meta.location !== LOCATIONS[a]) throw new Error(`velocity component ${a} is not on ${LOCATIONS[a]}`);
    const expected = n.map((c, b) => c + (a === b ? 1 : 0));
    if (p.meta.shape.some((s, b) => s !== expected[b])) throw new Error("velocity component shape does not match the grid");
    if (p.meta.frame.index !== u.meta.frame.index) throw new Error("velocity components come from different frames");
  });
  if (solid.length !== n[0] * n[1] * n[2]) throw new Error("solid mask does not match the grid");
  return {
    n,
    h: [...g.spacing] as Vec3,
    origin: [...g.origin] as Vec3,
    comp: [u.values, v.values, w.values],
    valid: [u.valid, v.valid, w.valid],
    solid,
  };
}

/** Samples along `axis` of component `c`: n + 1 on its own (staggered) axis, n otherwise. */
const samples = (f: MacVelocity, c: number, axis: number) => f.n[axis] + (c === axis ? 1 : 0);

/**
 * Interpolates component `c` at (x, y, z). Returns NaN when no valid sample is in the stencil.
 */
export function sampleComponent(f: MacVelocity, c: number, x: number, y: number, z: number): number {
  const p = [x, y, z];
  const i0 = [0, 0, 0];
  const t = [0, 0, 0];
  for (let a = 0; a < 3; a++) {
    const m = samples(f, c, a);
    let s = (p[a] - f.origin[a]) / f.h[a] - (c === a ? 0 : 0.5);
    if (s < 0) s = 0;
    if (s > m - 1) s = m - 1;
    let i = Math.floor(s);
    if (i > m - 2) i = Math.max(m - 2, 0);
    i0[a] = i;
    t[a] = m > 1 ? s - i : 0;
  }
  const mx = samples(f, c, 0);
  const my = samples(f, c, 1);
  const values = f.comp[c];
  const valid = f.valid[c];
  let sum = 0;
  let wsum = 0;
  for (let corner = 0; corner < 8; corner++) {
    const dx = corner & 1, dy = (corner >> 1) & 1, dz = (corner >> 2) & 1;
    const wgt = (dx ? t[0] : 1 - t[0]) * (dy ? t[1] : 1 - t[1]) * (dz ? t[2] : 1 - t[2]);
    if (wgt === 0) continue;
    const idx = i0[0] + dx + mx * (i0[1] + dy + my * (i0[2] + dz));
    if (!valid[idx]) continue;
    const val = values[idx];
    if (!Number.isFinite(val)) return NaN;
    sum += wgt * val;
    wsum += wgt;
  }
  return wsum > 1e-12 ? sum / wsum : NaN;
}

/** Writes the velocity at p into out; false when any component is unavailable. */
export function sampleVelocity(f: MacVelocity, p: Vec3, out: Vec3): boolean {
  for (let c = 0; c < 3; c++) {
    const val = sampleComponent(f, c, p[0], p[1], p[2]);
    if (!Number.isFinite(val)) return false;
    out[c] = val;
  }
  return true;
}

export function insideDomain(f: GridGeometry, p: Vec3): boolean {
  for (let a = 0; a < 3; a++) {
    const lo = f.origin[a];
    if (!(p[a] >= lo && p[a] <= lo + f.n[a] * f.h[a])) return false;
  }
  return true;
}

/** Cell containing p (clamped to the grid), as a linear index. */
export function cellIndex(f: GridGeometry, p: Vec3): number {
  const idx = [0, 1, 2].map((a) => Math.min(Math.max(Math.floor((p[a] - f.origin[a]) / f.h[a]), 0), f.n[a] - 1));
  return idx[0] + f.n[0] * (idx[1] + f.n[1] * idx[2]);
}

export function inSolid(f: GridGeometry, p: Vec3): boolean {
  return f.solid[cellIndex(f, p)] === 1;
}

/**
 * True when the straight segment a -> b passes only through fluid cells (3D DDA over every cell
 * the segment touches, so it cannot cut a solid corner).
 */
export function segmentInFluid(f: GridGeometry, a: Vec3, b: Vec3): boolean {
  const cell = [0, 1, 2].map((k) => Math.min(Math.max(Math.floor((a[k] - f.origin[k]) / f.h[k]), 0), f.n[k] - 1));
  const end = [0, 1, 2].map((k) => Math.min(Math.max(Math.floor((b[k] - f.origin[k]) / f.h[k]), 0), f.n[k] - 1));
  const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const step = [0, 0, 0];
  const tMax = [Infinity, Infinity, Infinity];
  const tDelta = [Infinity, Infinity, Infinity];
  for (let k = 0; k < 3; k++) {
    if (d[k] > 0) {
      step[k] = 1;
      tMax[k] = (f.origin[k] + (cell[k] + 1) * f.h[k] - a[k]) / d[k];
      tDelta[k] = f.h[k] / d[k];
    } else if (d[k] < 0) {
      step[k] = -1;
      tMax[k] = (f.origin[k] + cell[k] * f.h[k] - a[k]) / d[k];
      tDelta[k] = -f.h[k] / d[k];
    }
  }
  const at = () => cell[0] + f.n[0] * (cell[1] + f.n[1] * cell[2]);
  if (f.solid[at()]) return false;
  for (let guard = 0; guard < 64; guard++) {
    if (cell[0] === end[0] && cell[1] === end[1] && cell[2] === end[2]) return true;
    const k = tMax[0] < tMax[1] ? (tMax[0] < tMax[2] ? 0 : 2) : tMax[1] < tMax[2] ? 1 : 2;
    if (tMax[k] > 1) return true;
    cell[k] += step[k];
    if (cell[k] < 0 || cell[k] >= f.n[k]) return true; // leaving the grid is handled by the domain test
    tMax[k] += tDelta[k];
    if (f.solid[at()]) return false;
  }
  return true;
}
