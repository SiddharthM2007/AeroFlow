export type Vec3 = [number, number, number];

export interface Domain {
  lx: number;
  ly: number;
  lz: number;
}

export interface Resolution {
  nx: number;
  ny: number;
  nz: number;
}

export interface TestBox {
  min: Vec3;
  max: Vec3;
}

export interface ModelTransform {
  position: Vec3;
  /** Euler XYZ, degrees. */
  rotation: Vec3;
  scale: Vec3;
}

export type GizmoMode = "translate" | "rotate" | "scale" | "none";

export interface DisplayOptions {
  showLattice: boolean;
  showSolid: boolean;
  showFluid: boolean;
  showTestBox: boolean;
  slice: { enabled: boolean; axis: 0 | 1 | 2; index: number };
}
