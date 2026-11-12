import { describe, expect, it } from "vitest";
// Golden AFFIELD file written by the C++ engine (engine/tests/test_fields.cpp), shared with the backend.
import goldenDataUrl from "../../../../fixtures/fields/golden_v1.affield?inline";
import {
  colormapRGB,
  cssGradient,
  defaultColormap,
  defaultRange,
  legendTicks,
  normalize,
  sliceToRGBA,
} from "./colormap";
import { FieldFormatError, decodeField, encodeField, expectedShape, type FieldMetadata, type ScalarFieldData } from "./format";
import { extractSlice, fieldIndex, sampleExtent, samplePosition, sliceCorners, staggerOffset } from "./sampling";

function goldenBuffer(): ArrayBuffer {
  const base64 = goldenDataUrl.slice(goldenDataUrl.indexOf(",") + 1);
  return Uint8Array.from(atob(base64), (c) => c.charCodeAt(0)).buffer;
}

/** Synthetic field on a non-cubic 4 x 3 x 2 grid (dx 0.5, dy 0.2, dz 1.5): value = i + 10 j + 100 k. */
function synthetic(location: FieldMetadata["location"] = "cell", signed = true): ScalarFieldData {
  const resolution = { nx: 4, ny: 3, nz: 2 };
  const shape = expectedShape(location, resolution);
  const n = shape[0] * shape[1] * shape[2];
  const values = new Float32Array(n);
  const valid = new Uint8Array(n).fill(1);
  for (let k = 0; k < shape[2]; k++)
    for (let j = 0; j < shape[1]; j++)
      for (let i = 0; i < shape[0]; i++) values[fieldIndex(shape, i, j, k)] = i + 10 * j + 100 * k;
  valid[fieldIndex(shape, 1, 1, 0)] = 0;
  const meta: Omit<FieldMetadata, "blocks"> = {
    format: "aeroflow-field",
    version: 1,
    name: "synthetic",
    quantity: "test",
    long_name: "Synthetic",
    units: "m/s",
    signed,
    kind: "scalar",
    components: 1,
    location,
    shape,
    index_order: "i + n0*(j + n1*k)",
    byte_order: "little",
    grid: { resolution, origin: [0, 0, 0], spacing: [0.5, 0.2, 1.5], domain: [2, 0.6, 3] },
    frame: { index: 0, time_s: null, label: "test" },
    mask: {},
    provenance: { kind: "test", solved: false },
    stats: { valid_count: n - 1, min: -2, max: 8, max_abs: 8, mean_abs: 1, rms: 1 },
  };
  return decodeField(encodeField(meta, values, valid));
}

// --- binary format ----------------------------------------------------------------------------

describe("AFFIELD decoding", () => {
  it("decodes the engine's golden file x-fastest, without transposing", () => {
    const f = decodeField(goldenBuffer());
    expect(f.meta.version).toBe(1);
    expect(f.meta.shape).toEqual([3, 2, 2]);
    expect(f.meta.signed).toBe(false);
    for (let k = 0; k < 2; k++)
      for (let j = 0; j < 2; j++)
        for (let i = 0; i < 3; i++) {
          const idx = i + 3 * (j + 2 * k);
          if (i === 1 && j === 1 && k === 0) expect(f.valid[idx]).toBe(0);
          else expect(f.values[idx]).toBe(i + 10 * j + 100 * k + 0.25);
        }
    expect(f.meta.stats.valid_count).toBe(11);
  });

  it("round-trips through the TypeScript encoder", () => {
    const original = decodeField(goldenBuffer());
    const { blocks: _blocks, ...meta } = original.meta;
    const again = decodeField(encodeField(meta, original.values, original.valid));
    expect(Array.from(again.values)).toEqual(Array.from(original.values));
    expect(Array.from(again.valid)).toEqual(Array.from(original.valid));
  });

  it("rejects bad magic, versions, truncation and size mismatches", () => {
    const good = new Uint8Array(goldenBuffer());
    const mutate = (fn: (b: Uint8Array) => Uint8Array) => () => decodeField(fn(good.slice()).buffer as ArrayBuffer);
    expect(mutate((b) => ((b[0] = 88), b))).toThrow(/bad magic/);
    expect(mutate((b) => ((b[8] = 2), b))).toThrow(/unsupported AFFIELD version 2/);
    expect(mutate((b) => b.slice(0, 40))).toThrow(/truncated/);
    expect(mutate((b) => b.slice(0, b.length - 4))).toThrow(/outside the file/);
    const text = new TextDecoder().decode(good);
    const edit = (from: string, to: string) => () =>
      decodeField(new TextEncoder().encode(text.replace(from, to)).buffer as ArrayBuffer);
    expect(() => decodeField(new ArrayBuffer(4))).toThrow(FieldFormatError);
    // Same-length header edits keep offsets intact.
    expect(edit('"shape":[3,2,2]', '"shape":[3,2,3]')).toThrow(/does not match location/);
    expect(edit('"version":1', '"version":7')).toThrow(/wrong format or version/);
  });

  it("validates data length when encoding", () => {
    const f = synthetic();
    const { blocks: _b, ...meta } = f.meta;
    expect(() => encodeField(meta, f.values.subarray(1), f.valid)).toThrow(/does not match shape/);
  });
});

// --- coordinates and slicing ------------------------------------------------------------------

describe("sampling and slices", () => {
  it("places cell and staggered samples at their physical locations", () => {
    const cell = synthetic("cell").meta;
    expect(samplePosition(cell, 1, 2, 1)).toEqual([0.75, 0.5, 2.25]);
    const u = synthetic("face_x").meta;
    expect(u.shape).toEqual([5, 3, 2]);
    expect(staggerOffset("face_x", 0)).toBe(0);
    expect(staggerOffset("face_x", 1)).toBe(0.5);
    expect(samplePosition(u, 4, 0, 0)).toEqual([2, 0.1, 0.75]);
    expect(sampleExtent(cell, 0)).toEqual([0, 2]);
    expect(sampleExtent(u, 0)).toEqual([-0.25, 2.25]); // face samples' footprints straddle the walls
  });

  it("extracts X, Y and Z slices of a non-cubic grid in the documented texel order", () => {
    const f = synthetic();
    const x = extractSlice(f, 0, 2); // texel (a=j, b=k)
    expect([x.width, x.height, x.uAxis, x.vAxis]).toEqual([3, 2, 1, 2]);
    expect(x.values[2 + 3 * 1]).toBe(2 + 10 * 2 + 100 * 1);
    const y = extractSlice(f, 1, 1); // texel (a=i, b=k)
    expect([y.width, y.height]).toEqual([4, 2]);
    expect(y.values[3 + 4 * 1]).toBe(3 + 10 + 100);
    expect(y.valid[1 + 4 * 0]).toBe(0); // cell (1,1,0) invalid
    const z = extractSlice(f, 2, 1); // texel (a=i, b=j)
    expect([z.width, z.height]).toEqual([4, 3]);
    expect(z.values[1 + 4 * 2]).toBe(1 + 20 + 100);
    expect(() => extractSlice(f, 2, 2)).toThrow(RangeError);
  });

  it("positions slice planes physically inside the tunnel", () => {
    const f = synthetic();
    const s = extractSlice(f, 2, 1); // z-slice through cell centers k = 1 -> z = 2.25
    expect(s.planeCoordinate).toBe(2.25);
    const expected = [
      [0, 0, 2.25],
      [2, 0, 2.25],
      [2, 0.6, 2.25],
      [0, 0.6, 2.25],
    ];
    sliceCorners(s).forEach((c, n) => c.forEach((v, a) => expect(v).toBeCloseTo(expected[n][a], 12)));
    const x = extractSlice(f, 0, 0);
    expect(x.planeCoordinate).toBe(0.25);
    expect(x.uRange).toEqual([0, expect.closeTo(0.6, 12)]);
    expect(x.vRange).toEqual([0, 3]);
    const u = extractSlice(synthetic("face_x"), 0, 4); // u faces sit on x = i dx: last one on the wall
    expect(u.planeCoordinate).toBe(2);
  });
});

// --- colors -----------------------------------------------------------------------------------

describe("color mapping", () => {
  it("chooses maps and ranges from the field's signedness", () => {
    const signed = synthetic("cell", true).meta;
    expect(defaultColormap(signed)).toBe("diverging");
    expect(defaultRange(signed, "diverging")).toEqual({ min: -8, max: 8 });
    const positive = synthetic("cell", false).meta;
    expect(defaultColormap(positive)).toBe("sequential");
    expect(defaultRange(positive, "sequential")).toEqual({ min: -2, max: 8 });
  });

  it("centers signed values at zero and clamps out-of-range values", () => {
    const r = { min: -4, max: 4 };
    expect(normalize(0, r, "diverging")).toBe(0.5);
    expect(normalize(-4, r, "diverging")).toBe(0);
    expect(normalize(100, r, "diverging")).toBe(1);
    expect(normalize(-100, r, "diverging")).toBe(0);
    // Degenerate range (e.g. an exactly divergence-free field): neutral color, no NaN.
    expect(normalize(0, { min: 0, max: 0 }, "diverging")).toBe(0.5);
    expect(normalize(3, { min: 3, max: 3 }, "sequential")).toBe(0);
    expect(colormapRGB("diverging", 0.5)).toEqual([221, 221, 221]);
    const lo = colormapRGB("diverging", 0.1);
    const hi = colormapRGB("diverging", 0.9);
    expect(lo[2]).toBeGreaterThan(lo[0]); // negative: blue
    expect(hi[0]).toBeGreaterThan(hi[2]); // positive: red
  });

  it("masks invalid samples as fully transparent", () => {
    const s = extractSlice(synthetic(), 2, 0);
    const rgba = sliceToRGBA(s, { min: -8, max: 8 }, "diverging");
    const invalid = 1 + 4 * 1; // cell (1,1,0)
    expect(rgba[4 * invalid + 3]).toBe(0);
    expect(rgba[4 * 0 + 3]).toBe(255);
  });

  it("builds legend ticks and gradients", () => {
    expect(legendTicks({ min: -3, max: 3 })).toEqual([-3, 0, 3]);
    expect(legendTicks({ min: 0, max: 3 })).toEqual([0, 3]);
    expect(cssGradient("sequential", 4)).toMatch(/^linear-gradient\(to right, rgb\(68, 1, 84\) 0.0%/);
  });
});
