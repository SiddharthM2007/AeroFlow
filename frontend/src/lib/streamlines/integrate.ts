/**
 * Instantaneous streamlines (Milestone 8B): curves tangent to the velocity of ONE frame,
 * dx/ds = u(x) / |u(x)|, integrated with classical RK4 in arc length s with a fixed spatial step
 * (a fraction of the smallest cell size). In unsteady flow they are not particle paths.
 *
 * A streamline stops when the next point would leave the domain, when the step would enter or cut
 * through a solid cell (checked cell by cell), at stagnation (|u| below a fraction of the reference
 * speed), where velocity data is unavailable (no valid samples / non-finite), or when the length or
 * step budget is used up. The last point is always a fully checked fluid point.
 */
import { inSolid, insideDomain, sampleVelocity, segmentInFluid, type MacVelocity, type Vec3 } from "./velocityField";

export type StopReason = "domain" | "solid" | "stagnation" | "invalid" | "length" | "steps";
export type Direction = "forward" | "backward" | "both";

export interface TraceOptions {
  /** Step as a fraction of the smallest cell size (bounded spatial step). */
  stepFraction: number;
  maxLength: number; // m, per direction
  maxSteps: number; // per direction
  direction: Direction;
  /** Stagnation: |u| < stagnationFraction * referenceSpeed. */
  stagnationFraction: number;
  referenceSpeed: number;
}

export const DEFAULT_TRACE: Omit<TraceOptions, "maxLength" | "referenceSpeed"> = {
  stepFraction: 0.5,
  maxSteps: 4000,
  direction: "forward",
  stagnationFraction: 1e-3,
};

export interface Streamline {
  points: number[]; // x, y, z, ...
  speeds: number[]; // |u| at each point
  stops: StopReason[]; // one per integrated direction
}

function direction(f: MacVelocity, p: Vec3, sign: number, minSpeed: number, out: Vec3, speed: { v: number }): StopReason | null {
  if (!insideDomain(f, p)) return "domain";
  if (inSolid(f, p)) return "solid";
  if (!sampleVelocity(f, p, out)) return "invalid";
  const s = Math.hypot(out[0], out[1], out[2]);
  speed.v = s;
  if (!(s >= minSpeed) || s === 0) return "stagnation";
  out[0] = (sign * out[0]) / s;
  out[1] = (sign * out[1]) / s;
  out[2] = (sign * out[2]) / s;
  return null;
}

/** Integrates one direction from the seed; returns points after the seed and the stop reason. */
function traceOneWay(f: MacVelocity, seed: Vec3, sign: number, o: TraceOptions): { pts: number[]; spd: number[]; stop: StopReason } {
  const ds = o.stepFraction * Math.min(f.h[0], f.h[1], f.h[2]);
  const minSpeed = o.stagnationFraction * o.referenceSpeed;
  const pts: number[] = [];
  const spd: number[] = [];
  let p: Vec3 = [...seed] as Vec3;
  const k1: Vec3 = [0, 0, 0], k2: Vec3 = [0, 0, 0], k3: Vec3 = [0, 0, 0], k4: Vec3 = [0, 0, 0];
  const q: Vec3 = [0, 0, 0];
  const sp = { v: 0 };
  let length = 0;
  for (let n = 0; n < o.maxSteps; n++) {
    if (length + ds > o.maxLength * (1 + 1e-12)) return { pts, spd, stop: "length" };
    let r = direction(f, p, sign, minSpeed, k1, sp);
    if (r) return { pts, spd, stop: r };
    for (let a = 0; a < 3; a++) q[a] = p[a] + 0.5 * ds * k1[a];
    if ((r = direction(f, q, sign, minSpeed, k2, sp))) return { pts, spd, stop: r };
    for (let a = 0; a < 3; a++) q[a] = p[a] + 0.5 * ds * k2[a];
    if ((r = direction(f, q, sign, minSpeed, k3, sp))) return { pts, spd, stop: r };
    for (let a = 0; a < 3; a++) q[a] = p[a] + ds * k3[a];
    if ((r = direction(f, q, sign, minSpeed, k4, sp))) return { pts, spd, stop: r };
    const next: Vec3 = [0, 0, 0];
    for (let a = 0; a < 3; a++) next[a] = p[a] + (ds / 6) * (k1[a] + 2 * k2[a] + 2 * k3[a] + k4[a]);
    if (!insideDomain(f, next)) return { pts, spd, stop: "domain" };
    if (!segmentInFluid(f, p, next)) return { pts, spd, stop: "solid" };
    // Speed at the new point (for colouring); stop if it is stagnant or invalid there.
    const tmp: Vec3 = [0, 0, 0];
    if ((r = direction(f, next, sign, minSpeed, tmp, sp))) {
      if (r !== "stagnation") return { pts, spd, stop: r };
      pts.push(next[0], next[1], next[2]);
      spd.push(sp.v);
      return { pts, spd, stop: "stagnation" };
    }
    pts.push(next[0], next[1], next[2]);
    spd.push(sp.v);
    length += Math.hypot(next[0] - p[0], next[1] - p[1], next[2] - p[2]);
    p = next;
  }
  return { pts, spd, stop: "steps" };
}

/** Traces one streamline through `seed` (null if the seed itself is not a valid fluid point). */
export function traceStreamline(f: MacVelocity, seed: Vec3, o: TraceOptions): Streamline | null {
  const v: Vec3 = [0, 0, 0];
  if (!insideDomain(f, seed) || inSolid(f, seed) || !sampleVelocity(f, seed, v)) return null;
  const s0 = Math.hypot(v[0], v[1], v[2]);
  const points: number[] = [];
  const speeds: number[] = [];
  const stops: StopReason[] = [];
  if (o.direction !== "forward") {
    const back = traceOneWay(f, seed, -1, o);
    for (let n = back.spd.length - 1; n >= 0; n--) {
      points.push(back.pts[3 * n], back.pts[3 * n + 1], back.pts[3 * n + 2]);
      speeds.push(back.spd[n]);
    }
    stops.push(back.stop);
  }
  points.push(seed[0], seed[1], seed[2]);
  speeds.push(s0);
  if (o.direction !== "backward") {
    const fwd = traceOneWay(f, seed, 1, o);
    points.push(...fwd.pts);
    speeds.push(...fwd.spd);
    stops.push(fwd.stop);
  }
  return { points, speeds, stops };
}
