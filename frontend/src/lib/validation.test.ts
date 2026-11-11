import { describe, expect, it } from "vitest";
import { validateGridConfig } from "./validation";

const domain = { lx: 4, ly: 2, lz: 2 };
const resolution = { nx: 64, ny: 32, nz: 32 };
const box = { min: [1, 0, 0.75] as [number, number, number], max: [1.5, 0.5, 1.25] as [number, number, number] };

describe("validateGridConfig", () => {
  it("accepts a valid config", () => {
    expect(validateGridConfig(domain, resolution, box)).toEqual([]);
    expect(validateGridConfig(domain, resolution, null)).toEqual([]);
  });

  it("rejects non-positive lengths", () => {
    expect(validateGridConfig({ ...domain, lx: 0 }, resolution, null)).toContain("lx must be > 0");
  });

  it("rejects non-integer or out-of-range counts", () => {
    expect(validateGridConfig(domain, { ...resolution, ny: 2.5 }, null)[0]).toMatch(/^ny/);
    expect(validateGridConfig(domain, { ...resolution, nz: 0 }, null)[0]).toMatch(/^nz/);
    expect(validateGridConfig(domain, { ...resolution, nx: 513 }, null)[0]).toMatch(/^nx/);
  });

  it("rejects inverted and out-of-domain boxes", () => {
    expect(validateGridConfig(domain, resolution, { min: [2, 0, 0], max: [1, 1, 1] })).toContain(
      "test box min.x must be less than max.x",
    );
    expect(validateGridConfig(domain, resolution, { min: [0, 0, 0], max: [1, 3, 1] })).toContain(
      "test box y extent must lie within [0, 2]",
    );
  });
});
