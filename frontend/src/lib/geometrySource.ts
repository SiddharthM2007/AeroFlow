// Geometry source for the solid-cell mask (Milestone 2B). The procedural box and the imported
// mesh are mutually exclusive; a request never carries both.
import type { FieldId, GridJobRequest } from "./api";
import { boundsSize, serializeTransform, type Bounds, type LengthUnit } from "./transform";
import type { Domain, ModelTransform, Resolution, TestBox, Vec3 } from "./types";

export type GeometrySource = "procedural_box" | "mesh";

export const GEOMETRY_SOURCE_LABELS: Record<GeometrySource, string> = {
  procedural_box: "Procedural box",
  mesh: "Imported mesh",
};

export interface MeshInput {
  modelId: string;
  units: LengthUnit;
  pivot: Vec3;
  transform: ModelTransform;
}

export function buildGridJobRequest(
  source: GeometrySource,
  domain: Domain,
  resolution: Resolution,
  testBox: TestBox | null,
  mesh: MeshInput | null,
  fields: FieldId[] = [],
): GridJobRequest {
  const placement = mesh
    ? { model_id: mesh.modelId, model_transform: serializeTransform(mesh.units, mesh.pivot, mesh.transform) }
    : { model_id: null, model_transform: null };
  if (source === "mesh") {
    if (!mesh) throw new Error("imported mesh mode needs an uploaded model");
    return { domain, resolution, geometry_source: "mesh", test_box: null, ...placement, include_cells: true, fields };
  }
  return {
    domain,
    resolution,
    geometry_source: "procedural_box",
    test_box: testBox,
    ...placement,
    include_cells: true,
    fields,
  };
}

/** Cells spanned by the model's thinnest dimension; below this the shape is poorly resolved. */
export const MIN_CELLS_ACROSS = 4;

export interface ResolutionCheck {
  /** Model extent along each axis divided by the cell spacing on that axis. */
  cellsAcross: Vec3;
  minCellsAcross: number;
  poorlyResolved: boolean;
}

export function checkModelResolution(bounds: Bounds, spacing: Vec3): ResolutionCheck {
  const size = boundsSize(bounds);
  const cellsAcross = size.map((s, a) => s / spacing[a]) as Vec3;
  const minCellsAcross = Math.min(...cellsAcross);
  return { cellsAcross, minCellsAcross, poorlyResolved: minCellsAcross < MIN_CELLS_ACROSS };
}
