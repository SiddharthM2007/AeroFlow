// Sample positions and slicing for AFFIELD data. Works for any sampling location:
//   cell    : sample (i,j,k) at origin + ((i+.5)dx, (j+.5)dy, (k+.5)dz)
//   face_x  : sample (i,j,k) at origin + ( i    dx, (j+.5)dy, (k+.5)dz)   (MAC u)
//   face_y / face_z analogously (MAC v / w)
// Values are never resampled: a slice shows the field's own samples, each drawn over its own
// footprint [p - d/2, p + d/2] in the slice plane.
import type { Vec3 } from "../types";
import type { FieldLocation, FieldMetadata, ScalarFieldData } from "./format";

export type Axis = 0 | 1 | 2;
export const AXIS_NAMES = ["x", "y", "z"] as const;

/** In-plane (u, v) axes of a slice normal to `axis`. Texel (a, b) has a along u, b along v. */
export const SLICE_PLANE_AXES: Record<Axis, [Axis, Axis]> = { 0: [1, 2], 1: [0, 2], 2: [0, 1] };

/** 0 along the staggered axis of a face field, 0.5 otherwise (cell centers). */
export function staggerOffset(location: FieldLocation, axis: Axis): number {
  const staggered = location === "face_x" ? 0 : location === "face_y" ? 1 : location === "face_z" ? 2 : -1;
  return staggered === axis ? 0 : 0.5;
}

export function fieldIndex(shape: [number, number, number], i: number, j: number, k: number): number {
  return i + shape[0] * (j + shape[1] * k);
}

export function samplePosition(meta: FieldMetadata, i: number, j: number, k: number): Vec3 {
  const idx = [i, j, k];
  return [0, 1, 2].map(
    (a) => meta.grid.origin[a] + (idx[a] + staggerOffset(meta.location, a as Axis)) * meta.grid.spacing[a],
  ) as Vec3;
}

/** Physical extent covered by all samples along `axis` (sample footprints included). */
export function sampleExtent(meta: FieldMetadata, axis: Axis): [number, number] {
  const off = staggerOffset(meta.location, axis);
  const d = meta.grid.spacing[axis];
  const o = meta.grid.origin[axis];
  return [o + (off - 0.5) * d, o + (meta.shape[axis] - 1 + off + 0.5) * d];
}

export function sliceCount(meta: FieldMetadata, axis: Axis): number {
  return meta.shape[axis];
}

export interface ScalarSlice {
  axis: Axis;
  index: number;
  uAxis: Axis;
  vAxis: Axis;
  width: number; // samples along uAxis
  height: number; // samples along vAxis
  values: Float32Array; // texel (a, b) at a + width * b
  valid: Uint8Array;
  /** Physical coordinate of the slice plane along `axis`. */
  planeCoordinate: number;
  uRange: [number, number];
  vRange: [number, number];
}

export function extractSlice(field: ScalarFieldData, axis: Axis, index: number): ScalarSlice {
  const { meta } = field;
  const shape = meta.shape;
  if (!Number.isInteger(index) || index < 0 || index >= shape[axis]) {
    throw new RangeError(`slice index ${index} is outside 0..${shape[axis] - 1} along ${AXIS_NAMES[axis]}`);
  }
  const [uAxis, vAxis] = SLICE_PLANE_AXES[axis];
  const width = shape[uAxis];
  const height = shape[vAxis];
  const values = new Float32Array(width * height);
  const valid = new Uint8Array(width * height);
  const ijk = [0, 0, 0];
  ijk[axis] = index;
  for (let b = 0; b < height; b++) {
    ijk[vAxis] = b;
    for (let a = 0; a < width; a++) {
      ijk[uAxis] = a;
      const src = fieldIndex(shape, ijk[0], ijk[1], ijk[2]);
      values[a + width * b] = field.values[src];
      valid[a + width * b] = field.valid[src];
    }
  }
  return {
    axis,
    index,
    uAxis,
    vAxis,
    width,
    height,
    values,
    valid,
    planeCoordinate:
      meta.grid.origin[axis] + (index + staggerOffset(meta.location, axis)) * meta.grid.spacing[axis],
    uRange: sampleExtent(meta, uAxis),
    vRange: sampleExtent(meta, vAxis),
  };
}

/** Slice-plane corners (u0v0, u1v0, u1v1, u0v1) in world coordinates; texture uv = (0,0)..(1,1). */
export function sliceCorners(s: ScalarSlice): [Vec3, Vec3, Vec3, Vec3] {
  const corner = (u: number, v: number): Vec3 => {
    const p: Vec3 = [0, 0, 0];
    p[s.axis] = s.planeCoordinate;
    p[s.uAxis] = u;
    p[s.vAxis] = v;
    return p;
  };
  return [
    corner(s.uRange[0], s.vRange[0]),
    corner(s.uRange[1], s.vRange[0]),
    corner(s.uRange[1], s.vRange[1]),
    corner(s.uRange[0], s.vRange[1]),
  ];
}
