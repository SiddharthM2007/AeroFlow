import { describe, expect, it } from "vitest";
import { DEFAULT_TRACE, traceStreamline, type TraceOptions } from "./integrate";
import { inletSeeds, sliceRakeSeeds } from "./seeds";
import { makeField, type Fn } from "./testFields";
import { sampleComponent, segmentInFluid, type Vec3 } from "./velocityField";

const opts = (o: Partial<TraceOptions> = {}): TraceOptions => ({ ...DEFAULT_TRACE, maxLength: 100, referenceSpeed: 1, ...o });
const zero: Fn = () => 0;

describe("MAC velocity interpolation", () => {
  it("reproduces linear fields exactly on an anisotropic grid, at the true face locations", () => {
    const fu: Fn = (x, y, z) => 1 + 2 * x - 3 * y + 0.5 * z;
    const fv: Fn = (x, y, z) => -1 + x + 4 * y - z;
    const fw: Fn = (x, y, z) => 0.3 - x + y + 2 * z;
    const f = makeField([10, 6, 4], [0.1, 0.25, 0.4], fu, fv, fw);
    for (const p of [
      [0.33, 0.71, 0.9],
      [0.05, 0.2, 0.3],
      [0.9, 1.3, 1.4],
    ] as Vec3[]) {
      // Inside every component's sample hull the interpolation is exact.
      expect(sampleComponent(f, 0, ...p)).toBeCloseTo(fu(...p), 5);
      expect(sampleComponent(f, 1, ...p)).toBeCloseTo(fv(...p), 5);
      expect(sampleComponent(f, 2, ...p)).toBeCloseTo(fw(...p), 5);
    }
    // At a sample location the stored value is returned unchanged.
    expect(sampleComponent(f, 0, 0.3, 0.125, 0.2)).toBeCloseTo(f.comp[0][3 + 11 * (0 + 6 * 0)], 12);
  });

  it("uses only valid samples (renormalized) and reports no velocity when none are valid", () => {
    const f = makeField([8, 8, 8], [0.25, 0.25, 0.25], () => 2, zero, zero);
    // An invalid sample in the stencil (e.g. a face inside a solid) holds a bogus value: ignored.
    const idx = 3 + 9 * (3 + 8 * 3); // u face (3, 3, 3) at (0.75, 0.875, 0.875)
    f.comp[0][idx] = 99;
    f.valid[0][idx] = 0;
    expect(sampleComponent(f, 0, 0.8, 0.9, 0.95)).toBe(2);
    // Valid neighbours with different values are blended with renormalized weights.
    f.comp[0][4 + 9 * (3 + 8 * 3)] = 4; // u face (4, 3, 3) at x = 1.0, valid
    const v = sampleComponent(f, 0, 0.875, 0.875, 0.875); // halfway between faces 3 (invalid) and 4
    expect(v).toBe(4); // only face 4 is valid at this y, z
    // No valid sample in the stencil: no velocity.
    f.valid[0].fill(0);
    expect(Number.isNaN(sampleComponent(f, 0, 0.8, 0.9, 0.95))).toBe(true);
  });
});

describe("instantaneous streamlines", () => {
  it("are straight lines in uniform flow and stop at the domain boundary", () => {
    const f = makeField([20, 10, 10], [0.1, 0.1, 0.1], () => 3, zero, zero);
    const line = traceStreamline(f, [0.05, 0.43, 0.61], opts())!;
    expect(line.stops).toEqual(["domain"]);
    for (let n = 0; n < line.speeds.length; n++) {
      expect(line.points[3 * n + 1]).toBeCloseTo(0.43, 12);
      expect(line.points[3 * n + 2]).toBeCloseTo(0.61, 12);
      expect(line.speeds[n]).toBeCloseTo(3, 6);
    }
    const lastX = line.points[line.points.length - 3];
    expect(lastX).toBeGreaterThan(2 - 0.05 - 1e-9); // within one step (0.05) of the outlet
    expect(lastX).toBeLessThanOrEqual(2);
  });

  it("follow circles in solid-body rotation (RK4 closes the loop)", () => {
    const w = 2.0, xc = 1.0, yc = 1.0;
    const f = makeField([40, 40, 4], [0.05, 0.05, 0.1], (_x, y) => -w * (y - yc), (x) => w * (x - xc), zero);
    const r = 0.6;
    const seed: Vec3 = [xc + r, yc, 0.2];
    const line = traceStreamline(f, seed, opts({ maxLength: 2 * Math.PI * r, maxSteps: 100000 }))!;
    expect(line.stops).toEqual(["length"]);
    let maxRadiusError = 0;
    for (let n = 0; n < line.speeds.length; n++) {
      const dr = Math.hypot(line.points[3 * n] - xc, line.points[3 * n + 1] - yc) - r;
      maxRadiusError = Math.max(maxRadiusError, Math.abs(dr));
    }
    expect(maxRadiusError).toBeLessThan(1e-4 * r);
    // Counter-clockwise (w > 0): the first step moves towards +y.
    expect(line.points[4]).toBeGreaterThan(yc);
    // One full turn ends next to the seed (within one step).
    const end = line.points.slice(-3);
    expect(Math.hypot(end[0] - seed[0], end[1] - seed[1])).toBeLessThan(0.5 * 0.05);
    // Backward integration turns the other way.
    const back = traceStreamline(f, seed, opts({ maxLength: 0.3, direction: "backward" }))!;
    expect(back.points[1]).toBeLessThan(yc);
  });

  it("stop at stagnation and when the step or length budget runs out", () => {
    // u = -(x - 1): flow converges on the stagnation plane x = 1.
    const f = makeField([20, 4, 4], [0.1, 0.1, 0.1], (x) => -(x - 1), zero, zero);
    const line = traceStreamline(f, [1.8, 0.2, 0.2], opts({ referenceSpeed: 1, stagnationFraction: 1e-2 }))!;
    expect(line.stops).toEqual(["stagnation"]);
    const lastX = line.points[line.points.length - 3];
    // |u| < 1e-2 within 0.01 of x = 1; the tracer stops before any RK stage reaches that zone, so the
    // last point is within one step (0.05) of it.
    expect(lastX - 1).toBeGreaterThan(0);
    expect(lastX - 1).toBeLessThan(0.01 + 0.05 + 1e-12);
    const uniform = makeField([20, 4, 4], [0.1, 0.1, 0.1], () => 1, zero, zero);
    expect(traceStreamline(uniform, [0.1, 0.2, 0.2], opts({ maxSteps: 5 }))!.stops).toEqual(["steps"]);
    const short = traceStreamline(uniform, [0.1, 0.2, 0.2], opts({ maxLength: 0.42 }))!;
    expect(short.stops).toEqual(["length"]);
    expect(short.points[short.points.length - 3]).toBeCloseTo(0.1 + 0.4, 9); // 8 full steps of 0.05
    // Both directions: the seed sits between the two halves.
    const both = traceStreamline(uniform, [1.0, 0.2, 0.2], opts({ maxLength: 0.2, direction: "both" }))!;
    expect(both.stops).toEqual(["length", "length"]);
    expect(both.points[0]).toBeCloseTo(0.8, 9);
    expect(both.points[both.points.length - 3]).toBeCloseTo(1.2, 9);
  });

  it("never enter or cut through solid cells", () => {
    const solid: Vec3[] = [];
    for (let j = 0; j < 10; j++) for (let k = 0; k < 10; k++) solid.push([10, j, k]); // wall across x
    const f = makeField([20, 10, 10], [0.1, 0.1, 0.1], () => 1, zero, zero, solid);
    const line = traceStreamline(f, [0.05, 0.45, 0.45], opts())!;
    expect(line.stops).toEqual(["solid"]);
    for (let n = 0; n < line.speeds.length; n++) expect(line.points[3 * n]).toBeLessThan(1.0); // wall starts at x = 1
    // A diagonal step past a solid corner is caught even though both endpoints are in fluid.
    const corner = makeField([4, 4, 1], [1, 1, 1], () => 1, zero, zero, [[1, 1, 0]]);
    expect(segmentInFluid(corner, [0.9, 1.5, 0.5], [1.5, 0.9, 0.5])).toBe(false);
    expect(segmentInFluid(corner, [0.5, 0.5, 0.5], [0.9, 0.9, 0.5])).toBe(true);
    // A seed inside a solid gives no streamline.
    expect(traceStreamline(f, [1.05, 0.45, 0.45], opts())).toBeNull();
  });

  it("stop where velocity data is invalid", () => {
    const f = makeField([20, 4, 4], [0.1, 0.1, 0.1], () => 1, zero, zero);
    // Invalidate all u samples beyond x = 1.2 (e.g. data withheld): integration must stop there.
    for (let k = 0; k < 4; k++) for (let j = 0; j < 4; j++) for (let i = 13; i <= 20; i++) f.valid[0][i + 21 * (j + 4 * k)] = 0;
    const line = traceStreamline(f, [0.05, 0.2, 0.2], opts())!;
    expect(line.stops).toEqual(["invalid"]);
    expect(line.points[line.points.length - 3]).toBeLessThan(1.3);
  });
});

describe("seeds", () => {
  const f = makeField([20, 10, 8], [0.1, 0.1, 0.1], () => 1, zero, zero, [[2, 1, 1]]); // under seed (0.25, 0.125, 0.15)
  it("place a density x density lattice on the slice plane, skipping solid cells", () => {
    const seeds = sliceRakeSeeds(f, 2, 0.15, 4);
    expect(seeds).toHaveLength(15); // 16 lattice points, one inside the solid cell (2, 1, 1)
    expect(seeds.every((s) => s[2] === 0.15)).toBe(true);
    expect(seeds[0]).toEqual([0.75, 0.125, 0.15]); // (0.25, 0.125) is in the solid cell
    expect(sliceRakeSeeds(f, 0, 0.55, 3).every((s) => s[0] === 0.55)).toBe(true);
  });
  it("place inlet seeds half a cell inside the inlet and clamp the density", () => {
    const seeds = inletSeeds(f, 3);
    expect(seeds).toHaveLength(9);
    expect(seeds.every((s) => s[0] === 0.05)).toBe(true);
    expect(inletSeeds(f, 1000)).toHaveLength(32 * 32);
    expect(inletSeeds(f, 0)).toHaveLength(4);
  });
});

