// AFFIELD v1: the canonical binary numerical-field format shared with the C++ engine
// (engine/include/aeroflow/field_io.hpp) and the backend (backend/app/fields.py).
//
//   bytes 0..7    "AFFIELD\0"
//   bytes 8..11   uint32 LE version
//   bytes 12..15  uint32 LE JSON header length H
//   bytes 16..    UTF-8 JSON header (space-padded; blocks start 8-byte aligned)
//   blocks        "values" float32 LE and "valid" uint8, x-fastest over header.shape
import type { Resolution, Vec3 } from "../types";

export const FIELD_MAGIC = "AFFIELD\0";
export const FIELD_FORMAT_VERSION = 1;

export type FieldLocation = "cell" | "face_x" | "face_y" | "face_z";
export type Shape3 = [number, number, number];

export interface FieldStats {
  valid_count: number;
  min: number | null;
  max: number | null;
  max_abs: number | null;
  mean_abs: number | null;
  rms: number | null;
}

export interface FieldBlock {
  name: string;
  dtype: "float32" | "uint8";
  offset: number;
  length: number;
}

export interface FieldMetadata {
  format: "aeroflow-field";
  version: number;
  name: string;
  quantity: string;
  long_name: string;
  units: string;
  /** True when the quantity may be negative (use a zero-centered diverging map). */
  signed: boolean;
  kind: "scalar";
  components: number;
  location: FieldLocation;
  shape: Shape3;
  index_order: string;
  byte_order: "little";
  grid: { resolution: Resolution; origin: Vec3; spacing: Vec3; domain: Vec3 };
  frame: { index: number; time_s: number | null; label: string };
  mask: Record<string, unknown>;
  provenance: { kind: string; solved: boolean; description?: string; [key: string]: unknown };
  stats: FieldStats;
  blocks?: FieldBlock[];
}

export interface ScalarFieldData {
  meta: FieldMetadata;
  values: Float32Array;
  /** 1 = valid sample; 0 = invalid (e.g. solid cell). Same indexing as `values`. */
  valid: Uint8Array;
}

export class FieldFormatError extends Error {}

const LOCATION_EXTRA: Record<FieldLocation, Shape3> = {
  cell: [0, 0, 0],
  face_x: [1, 0, 0],
  face_y: [0, 1, 0],
  face_z: [0, 0, 1],
};

export function expectedShape(location: FieldLocation, r: Resolution): Shape3 {
  const extra = LOCATION_EXTRA[location];
  if (!extra) throw new FieldFormatError(`unknown field location '${location}'`);
  return [r.nx + extra[0], r.ny + extra[1], r.nz + extra[2]];
}

const LITTLE_ENDIAN_HOST = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;

function readFloat32LE(buffer: ArrayBuffer, offset: number, count: number): Float32Array {
  if (LITTLE_ENDIAN_HOST) return new Float32Array(buffer.slice(offset, offset + 4 * count));
  const view = new DataView(buffer, offset, 4 * count);
  const out = new Float32Array(count);
  for (let n = 0; n < count; n++) out[n] = view.getFloat32(4 * n, true);
  return out;
}

/** Decodes and validates an AFFIELD file. Throws FieldFormatError with a precise message. */
export function decodeField(buffer: ArrayBuffer): ScalarFieldData {
  if (buffer.byteLength < 16) throw new FieldFormatError("field file is truncated");
  const bytes = new Uint8Array(buffer);
  const magic = String.fromCharCode(...bytes.subarray(0, 8));
  if (magic !== FIELD_MAGIC) throw new FieldFormatError("not an AFFIELD file (bad magic)");
  const view = new DataView(buffer);
  const version = view.getUint32(8, true);
  if (version !== FIELD_FORMAT_VERSION) {
    throw new FieldFormatError(`unsupported AFFIELD version ${version} (expected ${FIELD_FORMAT_VERSION})`);
  }
  const headerLength = view.getUint32(12, true);
  if (16 + headerLength > buffer.byteLength) throw new FieldFormatError("field header is truncated");

  let meta: FieldMetadata;
  try {
    meta = JSON.parse(new TextDecoder().decode(bytes.subarray(16, 16 + headerLength)));
  } catch (e) {
    throw new FieldFormatError(`field header is not valid JSON: ${(e as Error).message}`);
  }
  if (meta.format !== "aeroflow-field" || meta.version !== FIELD_FORMAT_VERSION) {
    throw new FieldFormatError("field header has the wrong format or version");
  }
  if (meta.byte_order !== "little" || meta.index_order !== "i + n0*(j + n1*k)") {
    throw new FieldFormatError("field header has an unsupported byte or index order");
  }
  const shape = meta.shape;
  const expected = expectedShape(meta.location, meta.grid.resolution);
  if (!Array.isArray(shape) || shape.length !== 3 || shape.some((n, a) => n !== expected[a])) {
    throw new FieldFormatError(`field shape ${JSON.stringify(shape)} does not match location '${meta.location}'`);
  }
  const count = shape[0] * shape[1] * shape[2];

  const block = (name: string, dtype: FieldBlock["dtype"], bytesPer: number): FieldBlock => {
    const b = meta.blocks?.find((x) => x.name === name);
    if (!b) throw new FieldFormatError(`field file has no '${name}' block`);
    if (b.dtype !== dtype || b.length !== count * bytesPer) {
      throw new FieldFormatError(`field '${name}' block has the wrong type or length for shape ${shape.join("x")}`);
    }
    if (b.offset % 8 !== 0 || b.offset < 16 + headerLength || b.offset + b.length > buffer.byteLength) {
      throw new FieldFormatError(`field '${name}' block lies outside the file`);
    }
    return b;
  };
  const valuesBlock = block("values", "float32", 4);
  const validBlock = block("valid", "uint8", 1);
  return {
    meta,
    values: readFloat32LE(buffer, valuesBlock.offset, count),
    valid: bytes.slice(validBlock.offset, validBlock.offset + count),
  };
}

/** Encodes an AFFIELD v1 file (same layout as the C++ writer). Used by tests and tools. */
export function encodeField(meta: Omit<FieldMetadata, "blocks">, values: Float32Array, valid: Uint8Array): ArrayBuffer {
  const count = meta.shape[0] * meta.shape[1] * meta.shape[2];
  if (values.length !== count || valid.length !== count) throw new FieldFormatError("data length does not match shape");
  const align = (n: number) => Math.ceil(n / 8) * 8;
  let headerLength = 0;
  let text = "";
  for (let pass = 0; pass < 4; pass++) {
    const valuesOffset = 16 + headerLength;
    const validOffset = align(valuesOffset + 4 * count);
    text = JSON.stringify({
      ...meta,
      blocks: [
        { name: "values", dtype: "float32", offset: valuesOffset, length: 4 * count },
        { name: "valid", dtype: "uint8", offset: validOffset, length: count },
      ],
    });
    const padded = align(16 + new TextEncoder().encode(text).length) - 16;
    if (padded === headerLength) break;
    headerLength = padded;
  }
  const header = new TextEncoder().encode(text.padEnd(headerLength, " "));
  const valuesOffset = 16 + headerLength;
  const validOffset = align(valuesOffset + 4 * count);
  const out = new ArrayBuffer(validOffset + count);
  const bytes = new Uint8Array(out);
  const view = new DataView(out);
  for (let n = 0; n < 8; n++) bytes[n] = FIELD_MAGIC.charCodeAt(n);
  view.setUint32(8, FIELD_FORMAT_VERSION, true);
  view.setUint32(12, headerLength, true);
  bytes.set(header, 16);
  for (let n = 0; n < count; n++) view.setFloat32(valuesOffset + 4 * n, values[n], true);
  bytes.set(valid, validOffset);
  return out;
}
