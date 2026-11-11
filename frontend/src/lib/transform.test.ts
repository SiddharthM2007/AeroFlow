import { Group, Mesh, Vector3 } from "three";
import { describe, expect, it } from "vitest";
import { boundsSize, modelMatrix, serializeTransform, toRowMajor, worldBounds, UNIT_TO_METERS } from "./transform";
import type { ModelTransform, Vec3 } from "./types";

const identity: ModelTransform = { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };

// Raw 100 × 40 × 20 box (file units), min corner at (10, 0, -5).
const BOX_VERTS = (() => {
  const out: number[] = [];
  for (const x of [10, 110]) for (const y of [0, 40]) for (const z of [-5, 15]) out.push(x, y, z);
  return new Float32Array(out);
})();
const BOX_PIVOT: Vec3 = [60, 20, 5];

const expectVec = (actual: Vec3, expected: Vec3, digits = 9) =>
  actual.forEach((v, i) => expect(v).toBeCloseTo(expected[i], digits));

describe("unit conversion", () => {
  it("converts millimeters to meters", () => {
    const b = worldBounds(BOX_VERTS, modelMatrix("mm", BOX_PIVOT, identity));
    expectVec(boundsSize(b), [0.1, 0.04, 0.02]);
    expectVec(b.min, [-0.05, -0.02, -0.01]); // pivot maps to the origin
  });

  it("supports cm, m and inches", () => {
    expectVec(boundsSize(worldBounds(BOX_VERTS, modelMatrix("cm", BOX_PIVOT, identity))), [1, 0.4, 0.2]);
    expectVec(boundsSize(worldBounds(BOX_VERTS, modelMatrix("m", BOX_PIVOT, identity))), [100, 40, 20]);
    expectVec(boundsSize(worldBounds(BOX_VERTS, modelMatrix("in", BOX_PIVOT, identity))), [2.54, 1.016, 0.508]);
  });

  it("is never applied cumulatively when switching units", () => {
    const sizes = (["mm", "in", "m", "cm", "mm"] as const).map((u) =>
      boundsSize(worldBounds(BOX_VERTS, modelMatrix(u, BOX_PIVOT, identity))),
    );
    expect(sizes[4]).toEqual(sizes[0]);
    expect(sizes[2][0] / sizes[0][0]).toBeCloseTo(1000, 9);
  });
});

describe("world bounding box", () => {
  it("applies user scale on top of unit conversion", () => {
    const t = { ...identity, scale: [2, 1, 0.5] as Vec3 };
    expectVec(boundsSize(worldBounds(BOX_VERTS, modelMatrix("mm", BOX_PIVOT, t))), [0.2, 0.04, 0.01]);
  });

  it("applies translation to the pivot", () => {
    const t = { ...identity, position: [1, 2, 3] as Vec3 };
    const b = worldBounds(BOX_VERTS, modelMatrix("mm", BOX_PIVOT, t));
    expectVec(b.min, [0.95, 1.98, 2.99]);
    expectVec(b.max, [1.05, 2.02, 3.01]);
  });

  it("swaps extents for a 90° rotation about y", () => {
    const t = { ...identity, rotation: [0, 90, 0] as Vec3 };
    expectVec(boundsSize(worldBounds(BOX_VERTS, modelMatrix("mm", BOX_PIVOT, t))), [0.02, 0.04, 0.1]);
  });

  it("uses exact vertices, not the rotated raw box", () => {
    // A diamond (square rotated 45°) rotated back by −45° about z becomes an axis-aligned square.
    const r = Math.SQRT1_2;
    const diamond = new Float32Array([1, 0, 0, 0, 1, 0, -1, 0, 0, 0, -1, 0]);
    const t = { ...identity, rotation: [0, 0, -45] as Vec3 };
    const b = worldBounds(diamond, modelMatrix("m", [0, 0, 0], t));
    expectVec(boundsSize(b), [2 * r, 2 * r, 0], 6);
  });

  it("combines rotation, scale and translation in the order T·R·S·U·(p − pivot)", () => {
    const t: ModelTransform = { position: [2, 1, 0.5], rotation: [0, 0, 90], scale: [2, 1, 1] };
    // Raw (110, 0, -5): minus pivot → (50, −20, −10) mm → ×U → (0.05, −0.02, −0.01)
    // ×S → (0.1, −0.02, −0.01); Rz(90°): (x, y) → (−y, x) → (0.02, 0.1, −0.01); +T → (2.02, 1.1, 0.49)
    const m = modelMatrix("mm", BOX_PIVOT, t);
    expectVec(new Vector3(110, 0, -5).applyMatrix4(m).toArray() as Vec3, [2.02, 1.1, 0.49]);
  });

  it("matches the nested scene graph used for rendering", () => {
    const t: ModelTransform = { position: [0.3, -1, 2], rotation: [20, -35, 70], scale: [1.5, 0.7, 2] };
    const u = UNIT_TO_METERS.in;
    const outer = new Group();
    outer.position.set(...t.position);
    outer.rotation.set(...(t.rotation.map((d) => (d * Math.PI) / 180) as Vec3));
    outer.scale.set(...t.scale);
    const inner = new Mesh();
    inner.position.set(...(BOX_PIVOT.map((c) => -c * u) as Vec3));
    inner.scale.setScalar(u);
    outer.add(inner);
    outer.updateMatrixWorld(true);
    const expected = inner.matrixWorld.toArray();
    modelMatrix("in", BOX_PIVOT, t)
      .toArray()
      .forEach((v, i) => expect(v).toBeCloseTo(expected[i], 12));
  });
});

describe("serialization for the voxelizer", () => {
  // Same fixture as backend/tests/test_transform.py; both sides must agree.
  const t: ModelTransform = { position: [1, 2, 3], rotation: [90, 0, 0], scale: [2, 1, 1] };
  const EXPECTED_ROW_MAJOR = [0.002, 0, 0, 0.98, 0, 0, -0.001, 2.03, 0, 0.001, 0, 2.98, 0, 0, 0, 1];

  it("emits a row-major matrix consistent with the parameters", () => {
    const s = serializeTransform("mm", [10, 20, 30], t);
    expect(s.unit_scale).toBe(0.001);
    expect(s.rotation_deg).toEqual([90, 0, 0]);
    s.matrix.forEach((v, i) => expect(v).toBeCloseTo(EXPECTED_ROW_MAJOR[i], 12));
    expect(toRowMajor(modelMatrix("mm", [10, 20, 30], t))).toEqual(s.matrix);
  });
});
