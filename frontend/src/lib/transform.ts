// Model-to-world transform (approved for Milestone 2A).
//
//   p_world = T(position) · R(rotation) · S(scale) · U(unit) · (p_raw − pivot)
//
//   * p_raw     imported vertex coordinates, untouched, in file units
//   * pivot     raw bounding-box center (file units); rotation/scale pivot about it
//   * U         uniform unit conversion to meters (mm, cm, m, in)
//   * S         user scale, a deliberate physical transformation
//   * R         Euler XYZ in degrees, matrix R = Rx · Ry · Rz (Three.js "XYZ" order)
//   * T         position in tunnel coordinates (meters, flow +x, y up)
//
// The serialized form (`serializeTransform`) carries both the parameters and the composed
// 4×4 matrix (row-major), which is what the Milestone 2B voxelizer consumes.
import { Euler, Matrix4, Quaternion, Vector3 } from "three";
import type { ModelTransform, Vec3 } from "./types";

export type LengthUnit = "mm" | "cm" | "m" | "in";

export const UNIT_TO_METERS: Record<LengthUnit, number> = {
  mm: 0.001,
  cm: 0.01,
  m: 1,
  in: 0.0254,
};

export const UNIT_LABELS: Record<LengthUnit, string> = {
  mm: "Millimeters",
  cm: "Centimeters",
  m: "Meters",
  in: "Inches",
};

export interface Bounds {
  min: Vec3;
  max: Vec3;
}

/** Composes the raw-to-world matrix. Unit conversion is derived from `units` every time, never accumulated. */
export function modelMatrix(units: LengthUnit, pivot: Vec3, t: ModelTransform): Matrix4 {
  const u = UNIT_TO_METERS[units];
  const rotation = new Quaternion().setFromEuler(
    new Euler(...(t.rotation.map((d) => (d * Math.PI) / 180) as Vec3), "XYZ"),
  );
  const trs = new Matrix4().compose(
    new Vector3(...t.position),
    rotation,
    new Vector3(t.scale[0] * u, t.scale[1] * u, t.scale[2] * u),
  );
  return trs.multiply(new Matrix4().makeTranslation(-pivot[0], -pivot[1], -pivot[2]));
}

/** Row-major 16-element array (Three.js stores matrices column-major). */
export function toRowMajor(m: Matrix4): number[] {
  return m.clone().transpose().toArray();
}

/** Exact axis-aligned bounds of the transformed vertices (not of the transformed raw box). */
export function worldBounds(positions: ArrayLike<number>, m: Matrix4): Bounds {
  const e = m.elements; // column-major
  const min: Vec3 = [Infinity, Infinity, Infinity];
  const max: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (let n = 0; n + 2 < positions.length; n += 3) {
    const x = positions[n];
    const y = positions[n + 1];
    const z = positions[n + 2];
    const wx = e[0] * x + e[4] * y + e[8] * z + e[12];
    const wy = e[1] * x + e[5] * y + e[9] * z + e[13];
    const wz = e[2] * x + e[6] * y + e[10] * z + e[14];
    if (wx < min[0]) min[0] = wx;
    if (wy < min[1]) min[1] = wy;
    if (wz < min[2]) min[2] = wz;
    if (wx > max[0]) max[0] = wx;
    if (wy > max[1]) max[1] = wy;
    if (wz > max[2]) max[2] = wz;
  }
  return { min, max };
}

export function boundsSize(b: Bounds): Vec3 {
  return [b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]];
}

/** Payload sent to the backend with grid jobs (see backend/app/schemas.py ModelTransformSpec). */
export interface SerializedTransform {
  units: LengthUnit;
  unit_scale: number;
  pivot: Vec3;
  scale: Vec3;
  rotation_deg: Vec3;
  position: Vec3;
  matrix: number[];
}

export function serializeTransform(units: LengthUnit, pivot: Vec3, t: ModelTransform): SerializedTransform {
  return {
    units,
    unit_scale: UNIT_TO_METERS[units],
    pivot: [...pivot],
    scale: [...t.scale],
    rotation_deg: [...t.rotation],
    position: [...t.position],
    matrix: toRowMajor(modelMatrix(units, pivot, t)),
  };
}
