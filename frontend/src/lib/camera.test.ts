import { PerspectiveCamera, Vector3 } from "three";
import { describe, expect, it } from "vitest";
import { frameBounds, unionBounds } from "./camera";
import { DEFAULT_CLEARANCES, layoutTunnel } from "./fit";
import type { Bounds } from "./transform";

function cornersVisible(b: Bounds, fov: number, aspect: number): boolean {
  const f = frameBounds(b, fov, aspect);
  const cam = new PerspectiveCamera(fov, aspect, f.near, f.far);
  cam.position.set(...f.position);
  cam.lookAt(...f.target);
  cam.updateMatrixWorld(true);
  for (const x of [b.min[0], b.max[0]])
    for (const y of [b.min[1], b.max[1]])
      for (const z of [b.min[2], b.max[2]]) {
        const p = new Vector3(x, y, z).project(cam);
        if (Math.abs(p.x) > 1 || Math.abs(p.y) > 1 || p.z < -1 || p.z > 1) return false;
      }
  return true;
}

describe("frameBounds", () => {
  it("targets the box center", () => {
    const f = frameBounds({ min: [0, 0, 0], max: [4, 2, 2] }, 45, 1.6);
    expect(f.target).toEqual([2, 1, 1]);
  });

  it.each([0.5, 1, 1.78, 3])("keeps every corner in view at aspect %f", (aspect) => {
    expect(cornersVisible({ min: [0, 0, 0], max: [4, 2, 2] }, 45, aspect)).toBe(true);
    expect(cornersVisible({ min: [0, 0, 0], max: [120, 3, 0.5] }, 45, aspect)).toBe(true);
  });

  it("frames the tunnel and model after fitting", () => {
    const model: Bounds = { min: [10, 10, 10], max: [10.4, 10.1, 10.2] };
    const layout = layoutTunnel(model, [10.2, 10.05, 10.1], DEFAULT_CLEARANCES, [0.05, 0.05, 0.05]);
    const all = unionBounds({ min: [0, 0, 0], max: [layout.domain.lx, layout.domain.ly, layout.domain.lz] }, layout.modelBounds);
    expect(all.max).toEqual([layout.domain.lx, layout.domain.ly, layout.domain.lz]); // model is inside
    expect(cornersVisible(all, 45, 16 / 9)).toBe(true);
    expect(cornersVisible(layout.modelBounds, 45, 16 / 9)).toBe(true);
  });

  it("scales camera distance with box size (millimeter-scale and large models)", () => {
    const small = frameBounds({ min: [0, 0, 0], max: [0.01, 0.01, 0.01] }, 45, 1);
    const large = frameBounds({ min: [0, 0, 0], max: [100, 100, 100] }, 45, 1);
    const dist = (f: ReturnType<typeof frameBounds>) => Math.hypot(...f.position.map((p, i) => p - f.target[i]));
    expect(dist(large) / dist(small)).toBeCloseTo(10_000, 6);
    expect(small.near).toBeGreaterThan(0);
    expect(small.near).toBeLessThan(dist(small));
  });
});
