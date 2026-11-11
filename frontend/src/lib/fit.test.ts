import { describe, expect, it } from "vitest";
import {
  DEFAULT_CLEARANCES,
  clearanceDistances,
  currentSpacing,
  fitsLimits,
  insideTunnel,
  layoutTunnel,
  proposeFit,
  validateClearanceFactors,
} from "./fit";
import { boundsSize, modelMatrix, worldBounds, type Bounds } from "./transform";
import type { ModelTransform, Vec3 } from "./types";

const BIG_LIMITS = { maxCells: 1e12, maxCellsPerAxis: 1e6 };

// A 0.4 × 0.1 × 0.2 m model, currently centered at (1, 1, 1).
const bounds: Bounds = { min: [0.8, 0.95, 0.9], max: [1.2, 1.05, 1.1] };
const position: Vec3 = [1, 1, 1];
const spacing: Vec3 = [0.05, 0.05, 0.05];

describe("clearances", () => {
  it("are multiples of the largest world dimension D", () => {
    const c = clearanceDistances(bounds, DEFAULT_CLEARANCES);
    expect(c.D).toBeCloseTo(0.4, 12);
    expect(c.upstream).toBeCloseTo(0.8, 12);
    expect(c.downstream).toBeCloseTo(2.0, 12);
    expect(c.lateral).toBeCloseTo(0.6, 12);
    expect(c.top).toBeCloseTo(0.6, 12);
    expect(c.bottom).toBeCloseTo(0.6, 12);
  });

  it("rejects non-positive factors", () => {
    expect(validateClearanceFactors({ ...DEFAULT_CLEARANCES, top: 0 })).toEqual(["top clearance must be > 0"]);
  });
});

describe("layoutTunnel", () => {
  const layout = layoutTunnel(bounds, position, DEFAULT_CLEARANCES, spacing);

  it("sizes the tunnel as clearances + model, rounded up to whole cells", () => {
    // x: 0.8 + 0.4 + 2.0 = 3.2 → 64 cells; y: 0.6 + 0.1 + 0.6 = 1.3 → 26; z: 0.6 + 0.2 + 0.6 = 1.4 → 28
    expect(layout.resolution).toEqual({ nx: 64, ny: 26, nz: 28 });
    expect(layout.domain.lx).toBeCloseTo(3.2, 12);
    expect(layout.domain.ly).toBeCloseTo(1.3, 12);
    expect(layout.domain.lz).toBeCloseTo(1.4, 12);
    expect(layout.totalCells).toBe(64 * 26 * 28);
  });

  it("preserves cell spacing exactly (L = n · d)", () => {
    expect(layout.domain.lx / layout.resolution.nx).toBeCloseTo(0.05, 14);
    expect(layout.domain.ly / layout.resolution.ny).toBeCloseTo(0.05, 14);
    expect(layout.domain.lz / layout.resolution.nz).toBeCloseTo(0.05, 14);
  });

  it("translates the model without changing its size", () => {
    boundsSize(layout.modelBounds).forEach((s, a) => expect(s).toBeCloseTo(boundsSize(bounds)[a], 12));
    const delta = layout.position.map((p, a) => p - position[a]);
    layout.modelBounds.min.forEach((m, a) => expect(m - bounds.min[a]).toBeCloseTo(delta[a], 12));
  });

  it("leaves at least the requested clearance on every side", () => {
    const c = clearanceDistances(bounds, DEFAULT_CLEARANCES);
    const g = layout.gaps;
    expect(g.upstream).toBeGreaterThanOrEqual(c.upstream - 1e-12);
    expect(g.downstream).toBeGreaterThanOrEqual(c.downstream - 1e-12);
    expect(g.bottom).toBeGreaterThanOrEqual(c.bottom - 1e-12);
    expect(g.top).toBeGreaterThanOrEqual(c.top - 1e-12);
    expect(g.zMin).toBeGreaterThanOrEqual(c.lateral - 1e-12);
    expect(g.zMax).toBeGreaterThanOrEqual(c.lateral - 1e-12);
    expect(insideTunnel(layout.modelBounds, layout.domain)).toBe(true);
  });

  it("splits sub-cell slack evenly", () => {
    // y needs 1.3 m exactly at 26 cells → no slack; use 0.08 m cells: 1.3/0.08 = 16.25 → 17 cells, slack 0.06
    const l = layoutTunnel(bounds, position, DEFAULT_CLEARANCES, [0.08, 0.08, 0.08]);
    expect(l.resolution.ny).toBe(17);
    expect(l.gaps.bottom).toBeCloseTo(0.63, 12);
    expect(l.gaps.top).toBeCloseTo(0.63, 12);
  });

  it("keeps anisotropic spacing per axis", () => {
    const l = layoutTunnel(bounds, position, DEFAULT_CLEARANCES, [0.1, 0.05, 0.025]);
    expect(l.resolution).toEqual({ nx: 32, ny: 26, nz: 56 });
  });

  it("is idempotent: repeated fitting does not drift or rescale", () => {
    let pos = position;
    let b = bounds;
    let sp = spacing;
    let prev = layout;
    for (let n = 0; n < 5; n++) {
      const next = layoutTunnel(b, pos, DEFAULT_CLEARANCES, sp);
      expect(next.resolution).toEqual(layout.resolution);
      next.position.forEach((p, a) => expect(p).toBeCloseTo(prev.position[a], 12));
      boundsSize(next.modelBounds).forEach((s, a) => expect(s).toBeCloseTo(boundsSize(bounds)[a], 12));
      prev = next;
      pos = next.position;
      b = next.modelBounds;
      sp = currentSpacing(next.domain, next.resolution);
    }
  });

  it("recovers models outside or touching the tunnel walls", () => {
    for (const shift of [
      [-5, 0, 0],
      [0, -0.95, 0],
      [100, 20, -3],
    ] as Vec3[]) {
      const b: Bounds = { min: bounds.min.map((m, a) => m + shift[a]) as Vec3, max: bounds.max.map((m, a) => m + shift[a]) as Vec3 };
      const l = layoutTunnel(b, position, DEFAULT_CLEARANCES, spacing);
      expect(insideTunnel(l.modelBounds, l.domain)).toBe(true);
      expect(l.resolution).toEqual(layout.resolution);
    }
  });

  it("handles flat models (zero thickness on one axis)", () => {
    const flat: Bounds = { min: [0, 0, 0], max: [1, 0, 0.5] };
    const l = layoutTunnel(flat, [0.5, 0, 0.25], DEFAULT_CLEARANCES, [0.1, 0.1, 0.1]);
    expect(l.gaps.bottom).toBeCloseTo(1.5, 12);
    expect(insideTunnel(l.modelBounds, l.domain)).toBe(true);
  });

  it("rejects zero-extent models", () => {
    expect(() => layoutTunnel({ min: [1, 1, 1], max: [1, 1, 1] }, position, DEFAULT_CLEARANCES, spacing)).toThrow(/zero/);
  });
});

describe("proposeFit and resource limits", () => {
  it("accepts layouts within the budget", () => {
    const p = proposeFit(bounds, position, DEFAULT_CLEARANCES, spacing, BIG_LIMITS);
    expect(p.withinLimits).toBe(true);
    expect(p.coarser).toBeNull();
  });

  it("offers a coarser grid that fits the total-cell limit", () => {
    const limits = { maxCells: 10_000, maxCellsPerAxis: 512 };
    const p = proposeFit(bounds, position, DEFAULT_CLEARANCES, spacing, limits);
    expect(p.withinLimits).toBe(false);
    expect(p.required.totalCells).toBe(64 * 26 * 28); // untouched, for manual adjustment
    expect(p.coarser).not.toBeNull();
    const c = p.coarser!;
    expect(fitsLimits(c, limits)).toBe(true);
    expect(c.totalCells).toBeGreaterThan(limits.maxCells * 0.7); // not needlessly coarse
    // Uniform coarsening keeps the cell aspect ratio.
    expect(c.spacing[0] / c.spacing[1]).toBeCloseTo(1, 12);
    // Model is still fully inside with the requested clearances; its size is unchanged.
    expect(insideTunnel(c.modelBounds, c.domain)).toBe(true);
    expect(c.gaps.downstream).toBeGreaterThanOrEqual(p.clearances.downstream - 1e-12);
    boundsSize(c.modelBounds).forEach((s, a) => expect(s).toBeCloseTo(boundsSize(bounds)[a], 12));
  });

  it("respects the per-axis limit", () => {
    const limits = { maxCells: 1e9, maxCellsPerAxis: 40 };
    const p = proposeFit(bounds, position, DEFAULT_CLEARANCES, spacing, limits);
    expect(p.withinLimits).toBe(false);
    expect(Math.max(p.coarser!.resolution.nx, p.coarser!.resolution.ny, p.coarser!.resolution.nz)).toBeLessThanOrEqual(40);
  });
});

describe("end to end with a transformed CAD model", () => {
  it("fits a rotated, scaled millimeter model without changing its physical size", () => {
    // 2000 × 500 × 800 mm body, scaled ×1.5 in x, yawed 30°.
    const verts: number[] = [];
    for (const x of [0, 2000]) for (const y of [0, 500]) for (const z of [0, 800]) verts.push(x, y, z);
    const raw = new Float32Array(verts);
    const pivot: Vec3 = [1000, 250, 400];
    const t: ModelTransform = { position: [0.5, 0.2, 0.1], rotation: [0, 30, 0], scale: [1.5, 1, 1] };
    const before = worldBounds(raw, modelMatrix("mm", pivot, t));

    const p = proposeFit(before, t.position, DEFAULT_CLEARANCES, [0.1, 0.1, 0.1], BIG_LIMITS);
    const moved: ModelTransform = { ...t, position: p.required.position };
    const after = worldBounds(raw, modelMatrix("mm", pivot, moved));

    boundsSize(after).forEach((s, a) => expect(s).toBeCloseTo(boundsSize(before)[a], 9));
    after.min.forEach((m, a) => expect(m).toBeCloseTo(p.required.modelBounds.min[a], 9));
    expect(insideTunnel(after, p.required.domain)).toBe(true);
    expect(moved.scale).toEqual([1.5, 1, 1]);
    expect(moved.rotation).toEqual([0, 30, 0]);
  });
});
