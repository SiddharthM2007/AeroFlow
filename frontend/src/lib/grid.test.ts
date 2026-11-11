import { describe, expect, it } from "vitest";
import {
  CELL_SOLID,
  buildLattice,
  cellCenter,
  cellCentersOfType,
  cellIndex,
  decodeBase64,
  latticeSegmentCount,
  makeGridGeometry,
  unravelIndex,
} from "./grid";

const res = { nx: 3, ny: 4, nz: 5 };

describe("grid indexing (matches engine convention)", () => {
  it("is x-fastest", () => {
    expect(cellIndex(res, 0, 0, 0)).toBe(0);
    expect(cellIndex(res, 1, 0, 0)).toBe(1);
    expect(cellIndex(res, 0, 1, 0)).toBe(3);
    expect(cellIndex(res, 0, 0, 1)).toBe(12);
    expect(cellIndex(res, 2, 3, 4)).toBe(59);
  });

  it("round-trips", () => {
    for (let idx = 0; idx < 60; idx++) {
      const [i, j, k] = unravelIndex(res, idx);
      expect(cellIndex(res, i, j, k)).toBe(idx);
    }
  });
});

describe("cell centers", () => {
  it("are offset by half a cell", () => {
    const g = makeGridGeometry({ lx: 2, ly: 1, lz: 10 }, { nx: 4, ny: 2, nz: 5 });
    expect(cellCenter(g, 0, 0, 0)).toEqual([0.25, 0.25, 1]);
    expect(cellCenter(g, 3, 1, 4)).toEqual([1.75, 0.75, 9]);
  });
});

describe("classification decoding", () => {
  it("decodes the engine's base64 uint8 payload", () => {
    expect(Array.from(decodeBase64("AAEBAA=="))).toEqual([0, 1, 1, 0]);
  });

  it("selects solid cell centers", () => {
    const g = makeGridGeometry({ lx: 2, ly: 1, lz: 1 }, { nx: 2, ny: 1, nz: 1 });
    const centers = cellCentersOfType(g, new Uint8Array([0, 1]), CELL_SOLID);
    expect(Array.from(centers)).toEqual([1.5, 0.5, 0.5]);
  });
});

describe("lattice", () => {
  it("emits one segment per grid line", () => {
    const g = makeGridGeometry({ lx: 1, ly: 1, lz: 1 }, res);
    expect(buildLattice(g).length).toBe(latticeSegmentCount(res) * 6);
  });

  it("restricts to a slice layer", () => {
    const g = makeGridGeometry({ lx: 1, ly: 1, lz: 1 }, res);
    const slice = buildLattice(g, { axis: 2, index: 0 });
    // x-lines: (ny+1)*2, y-lines: (nx+1)*2, z-lines: (nx+1)*(ny+1)
    expect(slice.length / 6).toBe(5 * 2 + 4 * 2 + 4 * 5);
  });
});
