import { describe, expect, it } from "vitest";
import { Cancelled, FlowEngine, type PathlineResult, type TraceResult } from "./engine";
import { DEFAULT_TRACE } from "./integrate";
import {
  DEFAULT_PATH_OPTIONS,
  advanceInterval,
  newTracer,
  positionAt,
  substepsFor,
  temporalResolution,
  type FrameSample,
  type Tracer,
} from "./pathlines";
import { inletSeeds } from "./seeds";
import { makeField, type Fn } from "./testFields";
import type { MacVelocity, Vec3 } from "./velocityField";

const zero: Fn = () => 0;
const maxAbsOf = (f: MacVelocity): Vec3 =>
  [0, 1, 2].map((c) => f.comp[c].reduce((m, v) => Math.max(m, Math.abs(v)), 0)) as Vec3;

/** Integrates tracers through a sequence of frames (the engine's loop, without the worker). */
function run(frames: MacVelocity[], times: number[], seeds: Vec3[], o = DEFAULT_PATH_OPTIONS): Tracer[] {
  const tracers = seeds.map((s) => newTracer(s, times[0], frames[0])).filter((t): t is Tracer => t !== null);
  for (let k = 0; k + 1 < frames.length; k++) {
    const n = substepsFor(maxAbsOf(frames[k]), maxAbsOf(frames[k + 1]), frames[k].h, times[k + 1] - times[k], o);
    advanceInterval(tracers, frames[k], frames[k + 1], times[k], times[k + 1], n, o, k + 2 === frames.length);
  }
  return tracers;
}

const last = (t: Tracer): Vec3 => t.points.slice(-3) as Vec3;

describe("particle paths: numerical accuracy", () => {
  it("move uniformly in uniform flow and land exactly on the frame times", () => {
    const f = makeField([40, 10, 10], [0.1, 0.1, 0.1], () => 2, () => 0.5, zero);
    const times = [0, 0.1, 0.25, 0.4]; // uneven frame spacing, as after frame landing
    const [t] = run([f, f, f, f], times, [[0.5, 0.2, 0.5]]);
    expect(t.stop).toBe("end");
    expect(t.times[t.times.length - 1]).toBe(0.4);
    for (const time of times) expect(t.times).toContain(time);
    const p = last(t);
    expect(p[0]).toBeCloseTo(0.5 + 2 * 0.4, 10);
    expect(p[1]).toBeCloseTo(0.2 + 0.5 * 0.4, 10);
    expect(p[2]).toBeCloseTo(0.5, 12);
    expect(positionAt(t, 0.25)![0]).toBeCloseTo(0.5 + 2 * 0.25, 10);
  });

  it("integrate time-varying flow with the frames' timestamps (u = a + b t, exact for RK4)", () => {
    const a = 1, b = 4;
    const times = [0, 0.05, 0.1, 0.2];
    const frames = times.map((t) => makeField([60, 6, 6], [0.05, 0.1, 0.1], () => a + b * t, zero, zero));
    const [t] = run(frames, times, [[0.3, 0.3, 0.3]]);
    const x = (time: number) => 0.3 + a * time + 0.5 * b * time * time;
    // Exact up to the float32 storage of the velocity samples (as in AFFIELD files).
    for (const time of times) expect(Math.abs(positionAt(t, time)![0] - x(time))).toBeLessThan(1e-8);
    // A steady-flow streamline through the same seed would not follow this: the paths use time.
    expect(last(t)[0]).toBeGreaterThan(0.3 + a * 0.2 + 1e-3);
  });

  it("follow circles in steady rotation (RK4 with bounded substeps)", () => {
    const w = 2, xc = 1, yc = 1, r = 0.6;
    const f = makeField([40, 40, 4], [0.05, 0.05, 0.1], (_x, y) => -w * (y - yc), (x) => w * (x - xc), zero);
    const period = (2 * Math.PI) / w;
    const times = [0, period / 4, period / 2, (3 * period) / 4, period];
    const [t] = run([f, f, f, f, f], times, [[xc + r, yc, 0.2]]);
    let worst = 0;
    for (let n = 0; n < t.speeds.length; n++) {
      worst = Math.max(worst, Math.abs(Math.hypot(t.points[3 * n] - xc, t.points[3 * n + 1] - yc) - r));
    }
    expect(worst).toBeLessThan(1e-4 * r);
    const p = last(t); // one period: back at the seed
    expect(Math.hypot(p[0] - (xc + r), p[1] - yc)).toBeLessThan(1e-4 * r);
  });

  it("bound substeps per axis on anisotropic grids", () => {
    // 0.5 m/s along y with dy = 0.01 is 50 cells/s; 1 m/s along x with dx = 0.1 is 10 cells/s.
    const n = substepsFor([1, 0.5, 0], [1, 0.5, 0], [0.1, 0.01, 0.1], 0.1, DEFAULT_PATH_OPTIONS);
    expect(n).toBe(10); // 5 cells along y per interval / 0.5 cells per substep
    const f = makeField([20, 100, 4], [0.1, 0.01, 0.1], () => 1, () => 0.5, zero);
    const [t] = run([f, f], [0, 0.1], [[0.2, 0.2, 0.2]]);
    expect(t.speeds.length).toBe(11); // seed + 10 substeps
    expect(last(t)[1]).toBeCloseTo(0.25, 10);
  });

  it("never pass through solid geometry and stop at the domain boundary", () => {
    const wall: Vec3[] = [];
    for (let j = 0; j < 10; j++) for (let k = 0; k < 10; k++) wall.push([10, j, k]);
    const f = makeField([20, 10, 10], [0.1, 0.1, 0.1], () => 3, zero, zero, wall);
    // The wall face carries the stored zero normal velocity: the tracer decelerates and never
    // reaches the wall (no penetration), so it survives to the last frame in front of it.
    const [t] = run([f, f, f], [0, 0.3, 0.6], [[0.05, 0.45, 0.45]]);
    expect(t.stop).toBe("end");
    for (let n = 0; n < t.speeds.length; n++) expect(t.points[3 * n]).toBeLessThan(1.0);
    expect(last(t)[0]).toBeGreaterThan(0.9);
    // Safeguard: with inconsistent data (non-zero velocity into the wall face) the step that would
    // cross into the solid is refused.
    const leaky = makeField([20, 10, 10], [0.1, 0.1, 0.1], () => 3, zero, zero, wall);
    for (let k = 0; k < 10; k++) for (let j = 0; j < 10; j++) leaky.comp[0][10 + 21 * (j + 10 * k)] = 3;
    const [l] = run([leaky, leaky, leaky], [0, 0.3, 0.6], [[0.05, 0.45, 0.45]]);
    expect(l.stop).toBe("solid");
    for (let n = 0; n < l.speeds.length; n++) expect(l.points[3 * n]).toBeLessThan(1.0);
    const open = makeField([20, 10, 10], [0.1, 0.1, 0.1], () => 3, zero, zero);
    const [u] = run([open, open, open], [0, 0.5, 1.0], [[0.05, 0.45, 0.45]]);
    expect(u.stop).toBe("domain");
    expect(last(u)[0]).toBeLessThanOrEqual(2);
    expect(newTracer([1.05, 0.45, 0.45], 0, f)).toBeNull(); // seed inside a solid
  });
});

describe("temporal resolution safeguard", () => {
  const frame = (index: number, time: number, maxAbs: Vec3): FrameSample => ({ index, time, maxAbs });
  it("estimates displacement per exported frame per axis and classifies it", () => {
    const h: Vec3 = [0.1, 0.05, 0.1];
    const ok = temporalResolution([frame(0, 0, [5, 0, 0]), frame(1, 0.02, [5, 0, 0])], h);
    expect(ok.cellsPerFrame).toBeCloseTo(1, 12); // 5 m/s * 0.02 s / 0.1 m
    expect(ok.level).toBe("ok");
    const warn = temporalResolution([frame(0, 0, [5, 1, 0]), frame(1, 0.02, [5, 8, 0])], h);
    expect(warn.cellsPerFrame).toBeCloseTo(3.2, 12); // y: 8 m/s * 0.02 / 0.05
    expect(warn.level).toBe("warn");
    const bad = temporalResolution([frame(0, 0, [5, 0, 0]), frame(1, 0.02, [5, 0, 0]), frame(2, 0.2, [5, 0, 0])], h);
    expect(bad.level).toBe("withhold"); // 10 cells in the second interval
    expect(bad.worstInterval).toEqual([1, 2]);
    expect(bad.intervalForWithhold).toBeCloseTo(0.1, 12); // 5 cells at 50 cells/s
    expect(bad.intervalForWarn).toBeCloseTo(0.04, 12);
  });

  it("undersampled oscillating flow: substeps converge to the wrong path, which is why it is withheld", () => {
    // True flow: u = U sin(2 pi t / T) with period T; frames every T/2 sample it at zeros only.
    const U = 3, T = 0.2;
    const times = [0, T / 2, T, (3 * T) / 2];
    const frames = times.map((t) => makeField([60, 4, 4], [0.05, 0.1, 0.1], () => U * Math.sin((2 * Math.PI * t) / T), zero, zero));
    const fine = run(frames, times, [[1.0, 0.2, 0.2]], { ...DEFAULT_PATH_OPTIONS, substepCells: 0.01 });
    // Reconstruction says the tracer never moves (all frames are zero) ...
    expect(last(fine[0])[0]).toBeCloseTo(1.0, 12);
    // ... although the true tracer oscillates by U T / pi = 0.19 m (3.8 cells): substeps cannot help.
    const check = temporalResolution(
      times.map((t, i) => frame(i, t, [Math.abs(U * Math.sin((2 * Math.PI * t) / T)) + U, 0, 0])),
      [0.05, 0.1, 0.1],
    );
    expect(check.level).toBe("withhold");
  });
});

describe("flow engine (worker side)", () => {
  const uniform = makeField([20, 10, 10], [0.1, 0.1, 0.1], () => 1, zero, zero);
  const seeds = inletSeeds(uniform, 4);
  const frames = (n: number): FrameSample[] => Array.from({ length: n }, (_, i) => ({ index: i, time: 0.1 * i, maxAbs: [1, 0, 0] }));
  const streamOpts = { ...DEFAULT_TRACE, maxLength: 10, referenceSpeed: 1 };

  function engineWith(loader = async () => uniform, maxFrames = 3) {
    const loads: number[] = [];
    const engine = new FlowEngine(async () => undefined, 2_000_000, maxFrames);
    engine.configure("job", async (frame, signal) => {
      loads.push(frame);
      const v = await loader();
      if (signal.aborted) throw new DOMException("x", "AbortError");
      return v;
    });
    return { engine, loads };
  }

  it("computes particle paths over all frames with exact frame times and a bounded frame cache", async () => {
    const { engine, loads } = engineWith();
    const r = (await engine.pathlines({ jobId: 1, frames: frames(6), seeds, options: DEFAULT_PATH_OPTIONS })) as PathlineResult;
    expect(r.kind).toBe("pathlines");
    expect(loads).toEqual([0, 1, 2, 3, 4, 5]); // each frame loaded once, in order
    expect(engine.cachedFrames().length).toBeLessThanOrEqual(3); // never more than 3 frames retained
    expect(r.times[r.offsets[1] - 1]).toBeCloseTo(0.5, 12);
    expect(r.stops.end).toBe(seeds.length);
    expect(r.releaseFrame).toBe(0);
    expect(r.lastFrame).toBe(5);
  });

  it("rapid frame changes: only the latest streamline request completes", async () => {
    const gates: (() => void)[] = [];
    const { engine } = engineWith(() => new Promise((res) => gates.push(() => res(uniform))));
    const results = [1, 2, 3].map((jobId) => engine.streamlines({ jobId, frame: jobId, seeds, options: streamOpts }));
    gates.forEach((g) => g());
    const [a, b, c] = await Promise.all(results);
    expect(a).toBeNull();
    expect(b).toBeNull();
    expect((c as TraceResult).frame).toBe(3);
  });

  it("switching modes cancels the running computation and aborts its loads", async () => {
    let signalSeen: AbortSignal | null = null;
    const engine = new FlowEngine(async () => undefined);
    const gate: (() => void)[] = [];
    engine.configure("job", (_f, signal) => {
      signalSeen ??= signal;
      return new Promise((res) => gate.push(() => res(uniform)));
    });
    const paths = engine.pathlines({ jobId: 1, frames: frames(4), seeds, options: DEFAULT_PATH_OPTIONS });
    const lines = engine.streamlines({ jobId: 2, frame: 0, seeds, options: streamOpts });
    expect(signalSeen!.aborted).toBe(true); // the path job's pending load was aborted
    while (gate.length) gate.shift()!();
    await new Promise((r) => setTimeout(r, 0));
    while (gate.length) gate.shift()!();
    expect(await paths).toBeNull();
    expect((await lines)?.kind).toBe("streamlines");
  });

  it("evicts least recently used velocity frames and reloads them when needed again", async () => {
    const { engine, loads } = engineWith(async () => uniform, 2);
    for (const [jobId, frame] of [
      [1, 0],
      [2, 1],
      [3, 2],
      [4, 0],
    ]) {
      await engine.streamlines({ jobId, frame, seeds, options: streamOpts });
    }
    expect(loads).toEqual([0, 1, 2, 0]); // frame 0 was evicted by 2 and loaded again
    expect(engine.cachedFrames()).toEqual([2, 0]);
  });

  it("a new dataset (job) drops cached frames; Cancelled is an error type for superseded work", async () => {
    const { engine, loads } = engineWith();
    await engine.streamlines({ jobId: 1, frame: 0, seeds, options: streamOpts });
    engine.configure("other job", async () => uniform);
    expect(engine.cachedFrames()).toEqual([]);
    expect(loads).toEqual([0]);
    expect(new Cancelled()).toBeInstanceOf(Error);
  });

  it("bounds the total points of particle paths", async () => {
    const engine = new FlowEngine(async () => undefined, 100);
    engine.configure("job", async () => uniform);
    const r = (await engine.pathlines({ jobId: 1, frames: frames(6), seeds, options: DEFAULT_PATH_OPTIONS }))!;
    expect(r.truncated).toBe(true);
    expect(r.speeds.length).toBeLessThanOrEqual(100);
  });
});
