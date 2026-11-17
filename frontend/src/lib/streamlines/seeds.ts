/**
 * Streamline seeds (Milestone 8B). Seeds lie on a regular lattice; seeds in solid cells or outside
 * the domain are dropped.
 *   slice rake: on the current slice plane (axis, plane coordinate), `density` points per in-plane
 *               axis, at the centres of a density x density partition of the plane;
 *   inlet:      on the plane half a cell inside the inlet (x = x0 + dx/2), same lattice over (y, z).
 */
import { inSolid, insideDomain, type GridGeometry, type Vec3 } from "./velocityField";

export const MIN_DENSITY = 2;
export const MAX_DENSITY = 32; // at most 1,024 seeds

const clampDensity = (d: number) => Math.min(Math.max(Math.round(d), MIN_DENSITY), MAX_DENSITY);

function planeSeeds(f: GridGeometry, axis: 0 | 1 | 2, coordinate: number, density: number): Vec3[] {
  const n = clampDensity(density);
  const [ua, va] = axis === 0 ? [1, 2] : axis === 1 ? [0, 2] : [0, 1];
  const out: Vec3[] = [];
  for (let b = 0; b < n; b++)
    for (let a = 0; a < n; a++) {
      const p: Vec3 = [0, 0, 0];
      p[axis] = coordinate;
      p[ua] = f.origin[ua] + ((a + 0.5) / n) * f.n[ua] * f.h[ua];
      p[va] = f.origin[va] + ((b + 0.5) / n) * f.n[va] * f.h[va];
      if (insideDomain(f, p) && !inSolid(f, p)) out.push(p);
    }
  return out;
}

export function sliceRakeSeeds(f: GridGeometry, axis: 0 | 1 | 2, coordinate: number, density: number): Vec3[] {
  return planeSeeds(f, axis, coordinate, density);
}

export function inletSeeds(f: GridGeometry, density: number): Vec3[] {
  return planeSeeds(f, 0, f.origin[0] + 0.5 * f.h[0], density);
}
