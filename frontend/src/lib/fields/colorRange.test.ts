import { describe, expect, it } from "vitest";
import { allFramesRange, isInitialProjectionImpulse, rangeFrames } from "./colorRange";
import type { FieldMetadata } from "./format";

function frame(name: string, index: number, stats: { min: number; max: number }, step = index): FieldMetadata {
  return {
    name,
    frame: { index, time_s: index * 0.01, label: "" },
    provenance: { kind: "simulation", solved: true, step },
    stats: { ...stats, max_abs: Math.max(Math.abs(stats.min), Math.abs(stats.max)), rms: 0, mean_abs: 0, valid_count: 1 },
  } as unknown as FieldMetadata;
}

describe("all-frames colour range", () => {
  it("covers every frame of the field", () => {
    const frames = [frame("sim_velocity_magnitude", 0, { min: 0, max: 5 }), frame("sim_velocity_magnitude", 1, { min: 0.2, max: 6.6 })];
    expect(allFramesRange(frames, "sequential")).toEqual({ min: 0, max: 6.6 });
  });

  it("is symmetric about zero for diverging colour maps", () => {
    const frames = [frame("sim_u", 0, { min: -1, max: 7 }), frame("sim_u", 1, { min: -9, max: 2 })];
    expect(allFramesRange(frames, "diverging")).toEqual({ min: -9, max: 9 });
  });

  it("leaves out the initial-projection pressure impulse (frame 0)", () => {
    const frames = [
      frame("sim_pressure", 0, { min: -287, max: 287 }),
      frame("sim_pressure", 1, { min: -19, max: 18 }),
      frame("sim_pressure", 2, { min: -22, max: 17 }),
    ];
    expect(isInitialProjectionImpulse(frames[0])).toBe(true);
    expect(isInitialProjectionImpulse(frames[1])).toBe(false);
    expect(rangeFrames(frames)).toHaveLength(2);
    expect(allFramesRange(frames, "diverging")).toEqual({ min: -22, max: 22 });
    // Other fields keep frame 0 (the projected initial velocity is a real flow state).
    expect(isInitialProjectionImpulse(frame("sim_velocity_magnitude", 0, { min: 0, max: 8 }))).toBe(false);
  });

  it("falls back to frame 0 when it is the only frame", () => {
    expect(allFramesRange([frame("sim_pressure", 0, { min: -3, max: 2 })], "diverging")).toEqual({ min: -3, max: 3 });
    expect(allFramesRange([], "sequential")).toBeNull();
  });
});
