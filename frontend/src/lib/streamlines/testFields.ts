/** Analytic MAC velocity fields for tests (not used by the app). */
import type { MacVelocity, Vec3 } from "./velocityField";

export type Fn = (x: number, y: number, z: number) => number;

/** A MAC velocity with each component sampled from an analytic function at its true face positions. */
export function makeField(n: Vec3, h: Vec3, fu: Fn, fv: Fn, fw: Fn, solidCells: Vec3[] = []): MacVelocity {
  const fns = [fu, fv, fw];
  const comp = [0, 1, 2].map((c) => {
    const m = n.map((k, a) => k + (a === c ? 1 : 0));
    const arr = new Float32Array(m[0] * m[1] * m[2]);
    for (let k = 0; k < m[2]; k++)
      for (let j = 0; j < m[1]; j++)
        for (let i = 0; i < m[0]; i++) {
          const pos = [i, j, k].map((q, a) => (q + (a === c ? 0 : 0.5)) * h[a]);
          arr[i + m[0] * (j + m[1] * k)] = fns[c](pos[0], pos[1], pos[2]);
        }
    return arr;
  }) as [Float32Array, Float32Array, Float32Array];
  const solid = new Uint8Array(n[0] * n[1] * n[2]);
  for (const [i, j, k] of solidCells) solid[i + n[0] * (j + n[1] * k)] = 1;
  // Faces with a solid cell on both sides are invalid (as exported by the engine); faces between a
  // fluid and a solid cell carry zero normal velocity.
  const valid = [0, 1, 2].map((c) => {
    const m = n.map((k, a) => k + (a === c ? 1 : 0));
    const v = new Uint8Array(m[0] * m[1] * m[2]).fill(1);
    const isSolid = (q: number[]) => q.every((x, a) => x >= 0 && x < n[a]) && solid[q[0] + n[0] * (q[1] + n[1] * q[2])] === 1;
    for (let k = 0; k < m[2]; k++)
      for (let j = 0; j < m[1]; j++)
        for (let i = 0; i < m[0]; i++) {
          const hi = [i, j, k];
          const lo = [i, j, k];
          lo[c] -= 1;
          const idx = i + m[0] * (j + m[1] * k);
          const sLo = isSolid(lo), sHi = isSolid(hi);
          if (sLo && sHi) v[idx] = 0;
          else if (sLo || sHi) comp[c][idx] = 0;
        }
    return v;
  }) as [Uint8Array, Uint8Array, Uint8Array];
  return { n, h, origin: [0, 0, 0], comp, valid, solid };
}

