import type { Bounds } from "./transform";
import type { Vec3 } from "./types";

/** Default viewing direction (from target towards camera): upstream-left, above, from +z. */
export const DEFAULT_VIEW_DIR: Vec3 = [-0.6, 0.7, 1.3];

export interface CameraFrame {
  position: Vec3;
  target: Vec3;
  near: number;
  far: number;
}

export function unionBounds(a: Bounds, b: Bounds): Bounds {
  return {
    min: a.min.map((m, i) => Math.min(m, b.min[i])) as Vec3,
    max: a.max.map((m, i) => Math.max(m, b.max[i])) as Vec3,
  };
}

/**
 * Places a perspective camera so the whole box is visible: the bounding sphere is fitted
 * inside the narrower of the vertical/horizontal fields of view, with `margin` padding.
 */
export function frameBounds(
  b: Bounds,
  fovDeg: number,
  aspect: number,
  direction: Vec3 = DEFAULT_VIEW_DIR,
  margin = 1.1,
): CameraFrame {
  const target = b.min.map((m, i) => (m + b.max[i]) / 2) as Vec3;
  const radius = Math.max(Math.hypot(...b.max.map((m, i) => m - b.min[i])) / 2, 1e-6);
  const halfV = (fovDeg * Math.PI) / 360;
  const halfH = Math.atan(Math.tan(halfV) * aspect);
  const distance = (radius * margin) / Math.sin(Math.min(halfV, halfH));
  const len = Math.hypot(...direction);
  const position = target.map((t, i) => t + (direction[i] / len) * distance) as Vec3;
  return {
    position,
    target,
    near: Math.max(distance - radius * 2, distance * 1e-3),
    far: distance + radius * 10,
  };
}
