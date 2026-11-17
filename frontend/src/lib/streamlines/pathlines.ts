/**
 * Time-integrated particle paths (pathlines) of passive tracers (Milestone 8C):
 *
 *   dx/dt = u(x, t),  u(x, t) = (1 - s) u_k(x) + s u_{k+1}(x),  s = (t - t_k) / (t_{k+1} - t_k)
 *
 * between consecutive exported frames k, k+1 at their actual simulation times. Each frame is
 * interpolated in space exactly as for streamlines (velocityField.ts). Integration is classical RK4
 * in time with substeps that move a tracer at most `substepCells` cells along any axis (from the
 * frames' maximum speeds and the per-axis spacing); substeps end exactly on frame times, so tracer
 * positions are reported at the frames' timestamps.
 *
 * Substeps control the integration error of this reconstruction only. Linear interpolation in time
 * cannot recover flow changes between exported frames: if tracers move several cells per frame,
 * the reconstructed paths are not trustworthy however small the substeps (see temporalResolution).
 *
 * Not streaklines (no continuous release) and not streamlines (time-dependent velocity).
 */
import { insideDomain, inSolid, sampleVelocity, segmentInFluid, type MacVelocity, type Vec3 } from "./velocityField";

export type PathStop = "domain" | "solid" | "invalid" | "steps" | "end";

export const WARN_CELLS_PER_FRAME = 2;
export const WITHHOLD_CELLS_PER_FRAME = 5;

export interface FrameSample {
  index: number;
  time: number; // s
  /** Max |u|, |v|, |w| over the frame (AFFIELD header stats of sim_u, sim_v, sim_w). */
  maxAbs: Vec3;
}

export interface TemporalResolution {
  /** Largest estimated tracer displacement per exported frame interval, in cells (any axis). */
  cellsPerFrame: number;
  worstInterval: [number, number] | null; // frame indices
  level: "ok" | "warn" | "withhold";
  /** Frame interval that would keep the worst interval at the warning / withholding limits (s). */
  intervalForWarn: number | null;
  intervalForWithhold: number | null;
}

/**
 * Estimates how far tracers can move between exported frames: for each interval, the larger of the
 * two frames' max |u_a| times the interval, divided by the spacing h_a, maximized over axes. This is
 * an upper estimate of displacement, not an accuracy guarantee: flow can change within an interval
 * even when tracers move less than a cell.
 */
export function temporalResolution(frames: FrameSample[], h: Vec3): TemporalResolution {
  let worst = 0;
  let worstInterval: [number, number] | null = null;
  let rate = 0; // cells per second at the worst interval
  for (let k = 0; k + 1 < frames.length; k++) {
    const a = frames[k], b = frames[k + 1];
    const dt = b.time - a.time;
    if (!(dt > 0)) continue;
    const r = Math.max(...[0, 1, 2].map((ax) => Math.max(a.maxAbs[ax], b.maxAbs[ax]) / h[ax]));
    if (r * dt > worst) {
      worst = r * dt;
      worstInterval = [a.index, b.index];
      rate = r;
    }
  }
  const level = worst > WITHHOLD_CELLS_PER_FRAME ? "withhold" : worst > WARN_CELLS_PER_FRAME ? "warn" : "ok";
  return {
    cellsPerFrame: worst,
    worstInterval,
    level,
    intervalForWarn: rate > 0 ? WARN_CELLS_PER_FRAME / rate : null,
    intervalForWithhold: rate > 0 ? WITHHOLD_CELLS_PER_FRAME / rate : null,
  };
}

export interface PathOptions {
  /** Max tracer displacement per substep, in cells along any axis. */
  substepCells: number;
  /** Bound on substeps per frame interval (exceeding it stops the tracer with "steps"). */
  maxSubstepsPerInterval: number;
}

export const DEFAULT_PATH_OPTIONS: PathOptions = { substepCells: 0.5, maxSubstepsPerInterval: 400 };

export interface Tracer {
  points: number[]; // xyz per sample
  times: number[]; // s, per sample
  speeds: number[]; // |u| per sample
  alive: boolean;
  stop: PathStop | null;
}

export function newTracer(seed: Vec3, time: number, f0: MacVelocity): Tracer | null {
  const v: Vec3 = [0, 0, 0];
  if (!insideDomain(f0, seed) || inSolid(f0, seed) || !sampleVelocity(f0, seed, v)) return null;
  return { points: [...seed], times: [time], speeds: [Math.hypot(v[0], v[1], v[2])], alive: true, stop: null };
}

/** Velocity at (p, t) between two frames; false where either frame has no velocity. */
function velocityAt(f0: MacVelocity, f1: MacVelocity, s: number, p: Vec3, out: Vec3, tmp: Vec3): boolean {
  if (!insideDomain(f0, p) || inSolid(f0, p)) return false;
  if (!sampleVelocity(f0, p, out) || !sampleVelocity(f1, p, tmp)) return false;
  for (let a = 0; a < 3; a++) out[a] = (1 - s) * out[a] + s * tmp[a];
  return true;
}

/** Number of substeps for one interval (displacement per substep <= substepCells along every axis). */
export function substepsFor(maxAbs0: Vec3, maxAbs1: Vec3, h: Vec3, dt: number, o: PathOptions): number {
  const rate = Math.max(...[0, 1, 2].map((a) => Math.max(maxAbs0[a], maxAbs1[a]) / h[a]));
  return Math.max(1, Math.ceil((rate * dt) / o.substepCells));
}

/**
 * Advances live tracers from t0 (frame f0) to t1 (frame f1). Records every substep; the last sample
 * of a surviving tracer is exactly at t1.
 */
export function advanceInterval(
  tracers: Tracer[],
  f0: MacVelocity,
  f1: MacVelocity,
  t0: number,
  t1: number,
  substeps: number,
  o: PathOptions,
  isLast: boolean,
): void {
  if (substeps > o.maxSubstepsPerInterval) {
    for (const tr of tracers) if (tr.alive) Object.assign(tr, { alive: false, stop: "steps" as PathStop });
    return;
  }
  const dt = (t1 - t0) / substeps;
  const span = t1 - t0;
  const k1: Vec3 = [0, 0, 0], k2: Vec3 = [0, 0, 0], k3: Vec3 = [0, 0, 0], k4: Vec3 = [0, 0, 0];
  const q: Vec3 = [0, 0, 0], tmp: Vec3 = [0, 0, 0];
  for (const tr of tracers) {
    if (!tr.alive) continue;
    const n = tr.speeds.length - 1;
    let p: Vec3 = [tr.points[3 * n], tr.points[3 * n + 1], tr.points[3 * n + 2]];
    for (let m = 0; m < substeps; m++) {
      const ta = t0 + m * dt;
      const sa = (ta - t0) / span, sm = (ta + 0.5 * dt - t0) / span, sb = (ta + dt - t0) / span;
      let ok = velocityAt(f0, f1, sa, p, k1, tmp);
      if (ok) {
        for (let a = 0; a < 3; a++) q[a] = p[a] + 0.5 * dt * k1[a];
        ok = velocityAt(f0, f1, sm, q, k2, tmp);
      }
      if (ok) {
        for (let a = 0; a < 3; a++) q[a] = p[a] + 0.5 * dt * k2[a];
        ok = velocityAt(f0, f1, sm, q, k3, tmp);
      }
      if (ok) {
        for (let a = 0; a < 3; a++) q[a] = p[a] + dt * k3[a];
        ok = velocityAt(f0, f1, sb, q, k4, tmp);
      }
      if (!ok) {
        // Classify why: outside, solid or no data at one of the stage points.
        tr.alive = false;
        tr.stop = !insideDomain(f0, q) || !insideDomain(f0, p) ? "domain" : inSolid(f0, q) || inSolid(f0, p) ? "solid" : "invalid";
        break;
      }
      const next: Vec3 = [0, 0, 0];
      for (let a = 0; a < 3; a++) next[a] = p[a] + (dt / 6) * (k1[a] + 2 * k2[a] + 2 * k3[a] + k4[a]);
      if (!insideDomain(f0, next)) {
        tr.alive = false;
        tr.stop = "domain";
        break;
      }
      if (!segmentInFluid(f0, p, next)) {
        tr.alive = false;
        tr.stop = "solid";
        break;
      }
      const v: Vec3 = [0, 0, 0];
      const time = m + 1 === substeps ? t1 : ta + dt; // land exactly on the frame time
      if (!velocityAt(f0, f1, sb, next, v, tmp)) {
        tr.alive = false;
        tr.stop = "invalid";
        break;
      }
      tr.points.push(next[0], next[1], next[2]);
      tr.times.push(time);
      tr.speeds.push(Math.hypot(v[0], v[1], v[2]));
      p = next;
    }
    if (tr.alive && isLast) {
      tr.alive = false;
      tr.stop = "end";
    }
  }
}

/** Position of a tracer at time t (linear between recorded substeps), or null outside its life. */
export function positionAt(tr: Pick<Tracer, "points" | "times">, t: number): Vec3 | null {
  const ts = tr.times;
  if (ts.length === 0 || t < ts[0] || t > ts[ts.length - 1]) return null;
  let lo = 0, hi = ts.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] <= t) lo = mid;
    else hi = mid;
  }
  if (ts[hi] === t) lo = hi;
  const s = hi === lo ? 0 : (t - ts[lo]) / (ts[hi] - ts[lo]);
  const P = tr.points;
  return [0, 1, 2].map((a) => P[3 * lo + a] + s * (P[3 * Math.min(hi, ts.length - 1) + a] - P[3 * lo + a])) as Vec3;
}
