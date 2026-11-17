/**
 * Flow-visualization service (Milestone 8B/8C), run inside a Web Worker.
 *
 * - Velocity frames are obtained through an injected loader (the worker fetches and decodes the
 *   AFFIELD files itself, so the main thread neither copies nor retains velocity arrays) and kept in
 *   a small LRU of at most `maxFrames` frames: particle paths only ever need two adjacent frames.
 * - One request runs at a time. A newer request (either mode) cancels the running one at its next
 *   batch and aborts its pending loads.
 * - Output is bounded by a total point budget.
 */
import { traceStreamline, type StopReason, type TraceOptions } from "./integrate";
import { advanceInterval, newTracer, substepsFor, type FrameSample, type PathOptions, type PathStop, type Tracer } from "./pathlines";
import type { MacVelocity, Vec3 } from "./velocityField";

export const MAX_TOTAL_POINTS = 2_000_000;
export const BATCH = 16;
export const WORKER_FRAMES = 3;

export type VelocityLoader = (frame: number, signal: AbortSignal) => Promise<MacVelocity>;

export interface StreamlineRequest {
  jobId: number;
  frame: number;
  seeds: Vec3[];
  options: TraceOptions;
}

export interface PathlineRequest {
  jobId: number;
  /** Frames from the release frame to the last frame, in time order, with header max speeds. */
  frames: FrameSample[];
  seeds: Vec3[];
  options: PathOptions;
}

interface Lines {
  jobId: number;
  positions: Float32Array; // xyz per point
  speeds: Float32Array; // |u| per point
  offsets: Uint32Array; // first point of each line + end offset
  seeds: number;
  truncated: boolean;
  ms: number;
}

export interface TraceResult extends Lines {
  kind: "streamlines";
  frame: number; // the one velocity frame these curves are tangent to
  stops: Record<StopReason, number>;
}

export interface PathlineResult extends Lines {
  kind: "pathlines";
  times: Float64Array; // simulation time per point (s)
  releaseFrame: number;
  lastFrame: number;
  stops: Record<PathStop, number>;
}

export class Cancelled extends Error {
  constructor() {
    super("superseded");
  }
}

export class FlowEngine {
  private loader: VelocityLoader | null = null;
  private dataset: string | null = null;
  private readonly frames = new Map<number, Promise<MacVelocity>>(); // LRU (insertion order)
  private controller = new AbortController();
  private latestJob = 0;

  constructor(
    private readonly yieldToEvents: () => Promise<void> = () => new Promise((r) => setTimeout(r, 0)),
    private readonly maxPoints = MAX_TOTAL_POINTS,
    private readonly maxFrames = WORKER_FRAMES,
  ) {}

  /** A new dataset (job): drops cached frames and aborts pending loads. */
  configure(dataset: string, loader: VelocityLoader): void {
    if (dataset === this.dataset) return;
    this.controller.abort();
    this.controller = new AbortController();
    this.frames.clear();
    this.dataset = dataset;
    this.loader = loader;
  }

  /** Registers the newest request; older ones stop at their next batch and their loads abort. */
  announce(jobId: number): void {
    if (jobId <= this.latestJob) return;
    this.latestJob = jobId;
    this.controller.abort();
    this.controller = new AbortController();
    // Loads that were aborted must not stay cached.
    for (const [k, p] of this.frames) p.catch(() => this.frames.get(k) === p && this.frames.delete(k));
  }

  cachedFrames(): number[] {
    return [...this.frames.keys()];
  }

  private check(jobId: number): void {
    if (jobId !== this.latestJob) throw new Cancelled();
  }

  private async velocity(frame: number, jobId: number): Promise<MacVelocity> {
    if (!this.loader) throw new Error("no dataset configured");
    let p = this.frames.get(frame);
    if (p) {
      this.frames.delete(frame);
      this.frames.set(frame, p); // most recently used
    } else {
      const signal = this.controller.signal;
      p = this.loader(frame, signal);
      this.frames.set(frame, p);
      p.catch(() => this.frames.get(frame) === p && this.frames.delete(frame));
      while (this.frames.size > this.maxFrames) this.frames.delete(this.frames.keys().next().value as number);
    }
    const v = await p;
    this.check(jobId);
    return v;
  }

  /** Instantaneous streamlines of one frame. Null if superseded. */
  async streamlines(req: StreamlineRequest): Promise<TraceResult | null> {
    this.announce(req.jobId);
    try {
      const field = await this.velocity(req.frame, req.jobId);
      const start = performance.now();
      const positions: number[] = [], speeds: number[] = [], offsets: number[] = [];
      const stops: Record<StopReason, number> = { domain: 0, solid: 0, stagnation: 0, invalid: 0, length: 0, steps: 0 };
      let truncated = false;
      for (let b = 0; b < req.seeds.length && !truncated; b += BATCH) {
        if (b > 0) {
          await this.yieldToEvents();
          this.check(req.jobId);
        }
        for (const seed of req.seeds.slice(b, b + BATCH)) {
          const line = traceStreamline(field, seed, req.options);
          if (!line || line.speeds.length < 2) continue;
          if (speeds.length + line.speeds.length > this.maxPoints) {
            truncated = true;
            break;
          }
          offsets.push(speeds.length);
          positions.push(...line.points);
          speeds.push(...line.speeds);
          for (const s of line.stops) stops[s]++;
        }
      }
      offsets.push(speeds.length);
      return {
        kind: "streamlines",
        jobId: req.jobId,
        frame: req.frame,
        positions: Float32Array.from(positions),
        speeds: Float32Array.from(speeds),
        offsets: Uint32Array.from(offsets),
        stops,
        seeds: req.seeds.length,
        truncated,
        ms: performance.now() - start,
      };
    } catch (e) {
      if (e instanceof Cancelled || (e instanceof DOMException && e.name === "AbortError")) return null;
      throw e;
    }
  }

  /** Particle paths from the first to the last frame of the request. Null if superseded. */
  async pathlines(req: PathlineRequest): Promise<PathlineResult | null> {
    this.announce(req.jobId);
    try {
      if (req.frames.length < 2) throw new Error("particle paths need at least two frames");
      const start = performance.now();
      let f0 = await this.velocity(req.frames[0].index, req.jobId);
      const tracers: Tracer[] = [];
      for (const s of req.seeds) {
        const t = newTracer(s, req.frames[0].time, f0);
        if (t) tracers.push(t);
      }
      let points = tracers.length;
      let truncated = false;
      for (let k = 0; k + 1 < req.frames.length && !truncated; k++) {
        const a = req.frames[k], b = req.frames[k + 1];
        const f1 = await this.velocity(b.index, req.jobId);
        const substeps = substepsFor(a.maxAbs, b.maxAbs, f0.h, b.time - a.time, req.options);
        const live = tracers.filter((t) => t.alive).length;
        if (points + live * Math.min(substeps, req.options.maxSubstepsPerInterval) > this.maxPoints) {
          truncated = true; // stop at frame a rather than exceed the point budget
          break;
        }
        for (let i = 0; i < tracers.length; i += BATCH) {
          await this.yieldToEvents();
          this.check(req.jobId);
          advanceInterval(tracers.slice(i, i + BATCH), f0, f1, a.time, b.time, substeps, req.options, k + 2 === req.frames.length);
        }
        points = tracers.reduce((n, t) => n + t.speeds.length, 0);
        f0 = f1;
      }
      const stops: Record<PathStop, number> = { domain: 0, solid: 0, invalid: 0, steps: 0, end: 0 };
      const offsets: number[] = [];
      let n = 0;
      for (const t of tracers) {
        offsets.push(n);
        n += t.speeds.length;
        if (t.stop) stops[t.stop]++;
      }
      offsets.push(n);
      const positions = new Float32Array(3 * n), speeds = new Float32Array(n), times = new Float64Array(n);
      let o = 0;
      for (const t of tracers) {
        positions.set(t.points, 3 * o);
        speeds.set(t.speeds, o);
        times.set(t.times, o);
        o += t.speeds.length;
      }
      return {
        kind: "pathlines",
        jobId: req.jobId,
        positions,
        speeds,
        times,
        offsets: Uint32Array.from(offsets),
        stops,
        seeds: req.seeds.length,
        truncated,
        releaseFrame: req.frames[0].index,
        lastFrame: req.frames[req.frames.length - 1].index,
        ms: performance.now() - start,
      };
    } catch (e) {
      if (e instanceof Cancelled || (e instanceof DOMException && e.name === "AbortError")) return null;
      throw e;
    }
  }
}
