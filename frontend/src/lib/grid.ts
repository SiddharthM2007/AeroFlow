// Client-side mirror of the engine's grid conventions (see engine/include/aeroflow/grid3d.hpp):
//   origin at (0,0,0), spacing L/N, cell centers at (i + 0.5) * d,
//   linear index = i + nx * (j + ny * k).
import type { Domain, Resolution, Vec3 } from "./types";

export const CELL_FLUID = 0;
export const CELL_SOLID = 1;

export interface GridGeometry {
  resolution: Resolution;
  domain: Domain;
  spacing: Vec3;
}

export function makeGridGeometry(domain: Domain, resolution: Resolution): GridGeometry {
  return {
    domain,
    resolution,
    spacing: [domain.lx / resolution.nx, domain.ly / resolution.ny, domain.lz / resolution.nz],
  };
}

export function cellIndex(r: Resolution, i: number, j: number, k: number): number {
  return i + r.nx * (j + r.ny * k);
}

export function unravelIndex(r: Resolution, idx: number): Vec3 {
  const i = idx % r.nx;
  const rest = Math.floor(idx / r.nx);
  return [i, rest % r.ny, Math.floor(rest / r.ny)];
}

export function cellCenter(g: GridGeometry, i: number, j: number, k: number): Vec3 {
  return [(i + 0.5) * g.spacing[0], (j + 0.5) * g.spacing[1], (k + 0.5) * g.spacing[2]];
}

export function decodeBase64(data: string): Uint8Array {
  const binary = atob(data);
  const out = new Uint8Array(binary.length);
  for (let n = 0; n < binary.length; n++) out[n] = binary.charCodeAt(n);
  return out;
}

export function latticeSegmentCount(r: Resolution): number {
  return (r.ny + 1) * (r.nz + 1) + (r.nx + 1) * (r.nz + 1) + (r.nx + 1) * (r.ny + 1);
}

/**
 * Line-segment endpoints (pairs of xyz) for every cell face edge in the grid.
 * When `slice` is given, only the lattice in that single cell layer is produced.
 */
export function buildLattice(
  g: GridGeometry,
  slice?: { axis: 0 | 1 | 2; index: number },
): Float32Array {
  const n = [g.resolution.nx, g.resolution.ny, g.resolution.nz];
  const d = g.spacing;
  // Planes (node indices) to iterate along each axis.
  const range = (axis: number): number[] => {
    if (slice && slice.axis === axis) return [slice.index, slice.index + 1];
    return Array.from({ length: n[axis] + 1 }, (_, m) => m);
  };
  const out: number[] = [];
  for (let axis = 0; axis < 3; axis++) {
    // Lines parallel to `axis`, one per node in the two other axes.
    const a = (axis + 1) % 3;
    const b = (axis + 2) % 3;
    const lo = slice && slice.axis === axis ? slice.index * d[axis] : 0;
    const hi = slice && slice.axis === axis ? (slice.index + 1) * d[axis] : n[axis] * d[axis];
    for (const ia of range(a)) {
      for (const ib of range(b)) {
        const p0: number[] = [0, 0, 0];
        const p1: number[] = [0, 0, 0];
        p0[a] = p1[a] = ia * d[a];
        p0[b] = p1[b] = ib * d[b];
        p0[axis] = lo;
        p1[axis] = hi;
        out.push(...p0, ...p1);
      }
    }
  }
  return new Float32Array(out);
}

/** Centers of cells with the given classification, optionally restricted to one slice. */
export function cellCentersOfType(
  g: GridGeometry,
  cells: Uint8Array,
  type: number,
  slice?: { axis: 0 | 1 | 2; index: number },
): Float32Array {
  const { nx, ny, nz } = g.resolution;
  const out: number[] = [];
  for (let k = 0; k < nz; k++) {
    if (slice?.axis === 2 && k !== slice.index) continue;
    for (let j = 0; j < ny; j++) {
      if (slice?.axis === 1 && j !== slice.index) continue;
      for (let i = 0; i < nx; i++) {
        if (slice?.axis === 0 && i !== slice.index) continue;
        if (cells[i + nx * (j + ny * k)] === type) out.push(...cellCenter(g, i, j, k));
      }
    }
  }
  return new Float32Array(out);
}
