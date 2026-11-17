/**
 * Pure display logic for velocity visualizations (Milestone 8C): which results may be drawn for
 * the frame on screen, and how far each particle path has progressed at the displayed time.
 */
import type { FieldMetadata } from "../fields/format";
import type { PathlineResult, TraceResult } from "./engine";
import { positionAt, type FrameSample } from "./pathlines";
import type { GridGeometry, Vec3 } from "./velocityField";

export const VELOCITY_COMPONENTS = ["sim_u", "sim_v", "sim_w"] as const;

/**
 * Streamlines are tangent to one frame's velocity, so they are drawn only for that frame. A result
 * computed for another frame (e.g. while playback runs ahead of the worker) is never shown.
 */
export function streamlinesForFrame(result: TraceResult | null, frame: number): TraceResult | null {
  return result && result.frame === frame ? result : null;
}

/** Frames with all three velocity components exported, from `releaseFrame` on, in time order. */
export function pathlineFrames(fields: FieldMetadata[], releaseFrame: number): FrameSample[] {
  const byFrame = new Map<number, Partial<Record<string, FieldMetadata>>>();
  for (const f of fields) {
    if (!(VELOCITY_COMPONENTS as readonly string[]).includes(f.name)) continue;
    const entry = byFrame.get(f.frame.index) ?? {};
    entry[f.name] = f;
    byFrame.set(f.frame.index, entry);
  }
  const out: FrameSample[] = [];
  for (const [index, e] of [...byFrame.entries()].sort((a, b) => a[0] - b[0])) {
    if (index < releaseFrame) continue;
    const comps = VELOCITY_COMPONENTS.map((c) => e[c]);
    if (comps.some((c) => !c || c.frame.time_s === null)) continue;
    out.push({
      index,
      time: comps[0]!.frame.time_s as number,
      maxAbs: comps.map((c) => c!.stats.max_abs ?? 0) as Vec3,
    });
  }
  return out;
}

/** Grid geometry from a velocity component's header and the cell mask. */
export function gridGeometry(meta: FieldMetadata, solid: Uint8Array): GridGeometry {
  const g = meta.grid;
  return { n: [g.resolution.nx, g.resolution.ny, g.resolution.nz], h: [...g.spacing] as Vec3, origin: [...g.origin] as Vec3, solid };
}

/**
 * Segment indices of every path up to time t (inclusive) and the tracer positions at t.
 * Before the release time nothing is drawn; a tracer that stopped earlier keeps its full path but
 * has no marker.
 */
export function pathsAtTime(r: PathlineResult, t: number): { index: Uint32Array; markers: Float32Array; alive: number } {
  const segs: number[] = [];
  const markers: number[] = [];
  for (let l = 0; l + 1 < r.offsets.length; l++) {
    const start = r.offsets[l], end = r.offsets[l + 1];
    if (end - start < 1 || t < r.times[start]) continue;
    let lo = start, hi = end; // first sample with time > t
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (r.times[mid] <= t) lo = mid + 1;
      else hi = mid;
    }
    for (let p = start; p + 1 < lo; p++) segs.push(p, p + 1);
    const tracer = { points: Array.from(r.positions.subarray(3 * start, 3 * end)), times: Array.from(r.times.subarray(start, end)) };
    const pos = positionAt(tracer, t);
    // Marker at the exact position at t (between recorded substeps, at most half a cell from the
    // last drawn sample); none once the tracer has stopped.
    if (pos && t <= r.times[end - 1]) markers.push(...pos);
  }
  return { index: Uint32Array.from(segs), markers: Float32Array.from(markers), alive: markers.length / 3 };
}
