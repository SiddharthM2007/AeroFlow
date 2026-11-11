// "Fit Tunnel to Model" (Milestone 2A). Resizes the tunnel around the model's transformed
// bounding box and translates the model; the model's scale, rotation and units are never touched.
//
// Approved rules:
//   * Free-stream clearances, each a multiple of D = max(bx, by, bz) of the world-space bbox:
//       upstream (−x), downstream (+x), lateral (±z, each side), top (+y), bottom (−y).
//   * Spacing preserved exactly: n = ceil(L_required / d), L = n · d. The sub-cell slack is split
//     evenly on both sides of each axis, so actual clearances are >= the requested ones.
//   * Tunnel origin stays at (0,0,0); flow +x; y up.
import type { Domain, Resolution, Vec3 } from "./types";
import { boundsSize, type Bounds } from "./transform";

export interface ClearanceFactors {
  upstream: number;
  downstream: number;
  lateral: number;
  top: number;
  bottom: number;
}

export const DEFAULT_CLEARANCES: ClearanceFactors = {
  upstream: 2,
  downstream: 5,
  lateral: 1.5,
  top: 1.5,
  bottom: 1.5,
};

export interface GridLimits {
  maxCells: number;
  maxCellsPerAxis: number;
}

/** Requested clearance distances in meters. */
export interface ClearanceDistances {
  D: number;
  upstream: number;
  downstream: number;
  lateral: number;
  top: number;
  bottom: number;
}

export function clearanceDistances(bounds: Bounds, f: ClearanceFactors): ClearanceDistances {
  const D = Math.max(...boundsSize(bounds));
  return {
    D,
    upstream: f.upstream * D,
    downstream: f.downstream * D,
    lateral: f.lateral * D,
    top: f.top * D,
    bottom: f.bottom * D,
  };
}

export interface TunnelLayout {
  domain: Domain;
  resolution: Resolution;
  spacing: Vec3;
  totalCells: number;
  /** New model position (transform.position), meters. */
  position: Vec3;
  /** Model world bounds after the translation. */
  modelBounds: Bounds;
  /** Actual gap between the model bbox and each tunnel wall, meters. */
  gaps: { upstream: number; downstream: number; bottom: number; top: number; zMin: number; zMax: number };
}

export interface FitProposal {
  clearances: ClearanceDistances;
  /** Layout at the current cell spacing. */
  required: TunnelLayout;
  withinLimits: boolean;
  /** Uniformly coarsened layout that fits the limits; null when `required` already fits. */
  coarser: TunnelLayout | null;
}

// Guards ceil() against L/d landing a hair above an integer through rounding.
const CEIL_EPS = 1e-9;

export function validateClearanceFactors(f: ClearanceFactors): string[] {
  return (Object.keys(f) as (keyof ClearanceFactors)[])
    .filter((k) => !Number.isFinite(f[k]) || f[k] <= 0)
    .map((k) => `${k} clearance must be > 0`);
}

export function layoutTunnel(
  bounds: Bounds,
  position: Vec3,
  factors: ClearanceFactors,
  spacing: Vec3,
): TunnelLayout {
  const size = boundsSize(bounds);
  const c = clearanceDistances(bounds, factors);
  if (!(c.D > 0) || !Number.isFinite(c.D)) throw new Error("model has zero or invalid extent");
  if (!spacing.every((d) => d > 0 && Number.isFinite(d))) throw new Error("cell spacing must be positive");

  const lowGap: Vec3 = [c.upstream, c.bottom, c.lateral];
  const highGap: Vec3 = [c.downstream, c.top, c.lateral];
  const n = [0, 0, 0];
  const L: Vec3 = [0, 0, 0];
  const newMin: Vec3 = [0, 0, 0];
  for (let a = 0; a < 3; a++) {
    const required = lowGap[a] + size[a] + highGap[a];
    n[a] = Math.max(1, Math.ceil(required / spacing[a] - CEIL_EPS));
    L[a] = n[a] * spacing[a];
    const slack = Math.max(0, L[a] - required);
    newMin[a] = lowGap[a] + slack / 2;
  }

  const delta = newMin.map((m, a) => m - bounds.min[a]) as Vec3;
  const modelBounds: Bounds = {
    min: newMin,
    max: bounds.max.map((m, a) => m + delta[a]) as Vec3,
  };
  return {
    domain: { lx: L[0], ly: L[1], lz: L[2] },
    resolution: { nx: n[0], ny: n[1], nz: n[2] },
    spacing: [...spacing],
    totalCells: n[0] * n[1] * n[2],
    position: position.map((p, a) => p + delta[a]) as Vec3,
    modelBounds,
    gaps: {
      upstream: modelBounds.min[0],
      downstream: L[0] - modelBounds.max[0],
      bottom: modelBounds.min[1],
      top: L[1] - modelBounds.max[1],
      zMin: modelBounds.min[2],
      zMax: L[2] - modelBounds.max[2],
    },
  };
}

export function fitsLimits(layout: TunnelLayout, limits: GridLimits): boolean {
  const { nx, ny, nz } = layout.resolution;
  return layout.totalCells <= limits.maxCells && Math.max(nx, ny, nz) <= limits.maxCellsPerAxis;
}

export function proposeFit(
  bounds: Bounds,
  position: Vec3,
  factors: ClearanceFactors,
  spacing: Vec3,
  limits: GridLimits,
): FitProposal {
  const required = layoutTunnel(bounds, position, factors, spacing);
  const clearances = clearanceDistances(bounds, factors);
  if (fitsLimits(required, limits)) return { clearances, required, withinLimits: true, coarser: null };

  // Uniformly coarsen all spacings (keeps cell aspect ratios), starting from the analytic estimate.
  const { nx, ny, nz } = required.resolution;
  let factor = Math.max(
    Math.cbrt(required.totalCells / limits.maxCells),
    Math.max(nx, ny, nz) / limits.maxCellsPerAxis,
    1,
  );
  for (let iter = 0; iter < 200; iter++) {
    const candidate = layoutTunnel(bounds, position, factors, spacing.map((d) => d * factor) as Vec3);
    if (fitsLimits(candidate, limits)) return { clearances, required, withinLimits: false, coarser: candidate };
    factor *= 1.01;
  }
  return { clearances, required, withinLimits: false, coarser: null };
}

export function currentSpacing(domain: Domain, resolution: Resolution): Vec3 {
  return [domain.lx / resolution.nx, domain.ly / resolution.ny, domain.lz / resolution.nz];
}

/** True when the box lies strictly inside the tunnel (no contact with any wall). */
export function insideTunnel(b: Bounds, domain: Domain): boolean {
  const L = [domain.lx, domain.ly, domain.lz];
  return b.min.every((m) => m > 0) && b.max.every((m, a) => m < L[a]);
}
