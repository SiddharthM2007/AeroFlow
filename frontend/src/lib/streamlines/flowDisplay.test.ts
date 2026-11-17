import { describe, expect, it } from "vitest";
import type { FieldMetadata } from "../fields/format";
import type { PathlineResult, TraceResult } from "./engine";
import { pathlineFrames, pathsAtTime, streamlinesForFrame } from "./flowDisplay";

const trace = (frame: number): TraceResult =>
  ({ kind: "streamlines", frame, jobId: 1, positions: new Float32Array(), speeds: new Float32Array(), offsets: new Uint32Array([0]) }) as unknown as TraceResult;

describe("frame synchronization of streamlines", () => {
  it("shows a result only for the frame it was computed from", () => {
    expect(streamlinesForFrame(trace(3), 3)?.frame).toBe(3);
    expect(streamlinesForFrame(trace(2), 3)).toBeNull(); // playback ran ahead: hidden, never relabelled
    expect(streamlinesForFrame(null, 3)).toBeNull();
  });
});

function meta(name: string, index: number, time: number | null, maxAbs: number): FieldMetadata {
  return { name, frame: { index, time_s: time, label: "" }, stats: { max_abs: maxAbs } } as unknown as FieldMetadata;
}

describe("particle-path frames", () => {
  it("uses frames with all three components from the release frame on, in time order", () => {
    const fields = [
      ...[0, 1, 2, 3].flatMap((i) => [meta("sim_u", i, 0.1 * i, 5 + i), meta("sim_v", i, 0.1 * i, 1), meta("sim_w", i, 0.1 * i, 2)]),
      meta("sim_pressure", 4, 0.4, 10),
      meta("sim_u", 4, 0.4, 5), // v and w missing at frame 4
    ];
    const frames = pathlineFrames(fields, 1);
    expect(frames.map((f) => f.index)).toEqual([1, 2, 3]);
    expect(frames[0]).toEqual({ index: 1, time: 0.1, maxAbs: [6, 1, 2] });
  });
});

describe("particle paths at the displayed time", () => {
  // Two tracers: A has samples at t = 0, 0.1, 0.2 along x; B stops at t = 0.1.
  const r = {
    kind: "pathlines",
    positions: Float32Array.from([0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 1, 0, 1, 1, 0]),
    speeds: new Float32Array(5).fill(10),
    times: Float64Array.from([0, 0.1, 0.2, 0, 0.1]),
    offsets: Uint32Array.from([0, 3, 5]),
  } as unknown as PathlineResult;

  it("reveals each path up to t and places markers at the tracers' positions at t", () => {
    const at = pathsAtTime(r, 0.15);
    expect(Array.from(at.index)).toEqual([0, 1, 3, 4]); // A: segment 0-1; B: its whole path
    expect(at.alive).toBe(1); // B stopped at t = 0.1: no marker
    expect(Array.from(at.markers)).toEqual([1.5, 0, 0]);
  });

  it("shows whole paths at the last time and nothing before the release time", () => {
    expect(Array.from(pathsAtTime(r, 0.2).index)).toEqual([0, 1, 1, 2, 3, 4]);
    expect(pathsAtTime(r, 0.2).alive).toBe(1);
    const before = pathsAtTime(r, -0.01);
    expect(before.index.length).toBe(0);
    expect(before.alive).toBe(0);
    // Exactly at a frame time: the marker is the recorded sample.
    expect(Array.from(pathsAtTime(r, 0.1).markers)).toEqual([1, 0, 0, 1, 1, 0]);
  });
});
