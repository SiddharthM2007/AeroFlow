// Frontend <-> backend geometry consistency, using the fixture shared with
// backend/tests/test_mesh.py. Both sides must produce the same world-space bounds.
import { describe, expect, it } from "vitest";
import boxObj from "../../../fixtures/geometry/box_quads.obj?raw";
import { buildGridJobRequest, checkModelResolution } from "./geometrySource";
import { CELL_SOLID, cellCentersOfType, makeGridGeometry } from "./grid";
import { loadModelFile } from "./loaders";
import { boundsSize, modelMatrix, worldBounds } from "./transform";
import type { ModelTransform, Vec3 } from "./types";

const TRANSFORM: ModelTransform = { position: [1, 1, 1], rotation: [0, 90, 0], scale: [1, 2, 1] };
const EXPECTED_MIN: Vec3 = [0.95, 0.8, 0.8];
const EXPECTED_MAX: Vec3 = [1.05, 1.2, 1.2];

async function loadFixture() {
  return loadModelFile(new File([boxObj], "box_quads.obj"));
}

describe("shared geometry fixture", () => {
  it("parses like the backend (fan triangulation, negative indices)", async () => {
    const model = await loadFixture();
    expect(model.triangles).toBe(12);
    expect(model.pivot).toEqual([20, 10, 5]);
    expect(model.rawSize.toArray()).toEqual([40, 20, 10]);
  });

  it("has the same world bounds and physical size as the engine input", async () => {
    const model = await loadFixture();
    const b = worldBounds(model.geometry.getAttribute("position").array, modelMatrix("cm", model.pivot, TRANSFORM));
    b.min.forEach((v, i) => expect(v).toBeCloseTo(EXPECTED_MIN[i], 12));
    b.max.forEach((v, i) => expect(v).toBeCloseTo(EXPECTED_MAX[i], 12));
    boundsSize(b).forEach((v, i) => expect(v).toBeCloseTo([0.1, 0.4, 0.4][i], 12));
  });

  it("builds a mesh request with the transform and no test box", async () => {
    const model = await loadFixture();
    const req = buildGridJobRequest(
      "mesh",
      { lx: 2, ly: 2, lz: 2 },
      { nx: 40, ny: 40, nz: 40 },
      { min: [0.1, 0.1, 0.1], max: [0.2, 0.2, 0.2] }, // ignored in mesh mode
      { modelId: "abc", units: "cm", pivot: model.pivot, transform: TRANSFORM },
    );
    expect(req.geometry_source).toBe("mesh");
    expect(req.test_box).toBeNull();
    expect(req.model_id).toBe("abc");
    expect(req.model_transform?.units).toBe("cm");
    expect(req.model_transform?.pivot).toEqual([20, 10, 5]);
    expect(req.model_transform?.matrix).toHaveLength(16);
  });
});

describe("rendering the engine's mesh mask", () => {
  it("places solid cells exactly inside the model bounds", () => {
    // Mask the backend test expects for this fixture: i in {19, 20}, j and k in 16..23 (n = 40).
    const n = 40;
    const grid = makeGridGeometry({ lx: 2, ly: 2, lz: 2 }, { nx: n, ny: n, nz: n });
    const cells = new Uint8Array(n * n * n);
    for (let k = 16; k <= 23; k++) for (let j = 16; j <= 23; j++) for (const i of [19, 20]) cells[i + n * (j + n * k)] = 1;
    const centers = cellCentersOfType(grid, cells, CELL_SOLID);
    expect(centers.length / 3).toBe(128);
    for (let p = 0; p < centers.length; p += 3) {
      for (let a = 0; a < 3; a++) {
        expect(centers[p + a]).toBeGreaterThan(EXPECTED_MIN[a]);
        expect(centers[p + a]).toBeLessThan(EXPECTED_MAX[a]);
      }
    }
  });
});

describe("geometry source requests", () => {
  const domain = { lx: 1, ly: 1, lz: 1 };
  const res = { nx: 8, ny: 8, nz: 8 };
  const box = { min: [0.1, 0.1, 0.1] as Vec3, max: [0.5, 0.5, 0.5] as Vec3 };

  it("procedural mode sends the box and never the mesh source", () => {
    const req = buildGridJobRequest("procedural_box", domain, res, box, null);
    expect(req.geometry_source).toBe("procedural_box");
    expect(req.test_box).toEqual(box);
    expect(req.model_transform).toBeNull();
  });

  it("mesh mode requires an uploaded model", () => {
    expect(() => buildGridJobRequest("mesh", domain, res, null, null)).toThrow(/uploaded model/);
  });
});

describe("checkModelResolution", () => {
  it("flags models thinner than four cells", () => {
    const thin = checkModelResolution({ min: [0, 0, 0], max: [1, 1, 0.05] }, [0.02, 0.02, 0.02]);
    expect(thin.minCellsAcross).toBeCloseTo(2.5, 12);
    expect(thin.poorlyResolved).toBe(true);
    expect(checkModelResolution({ min: [0, 0, 0], max: [1, 1, 0.1] }, [0.02, 0.02, 0.02]).poorlyResolved).toBe(false);
  });
});
