import type { FieldMetadata } from "./fields/format";
import type { SerializedTransform } from "./transform";
import type { Domain, Resolution, TestBox } from "./types";

export interface EngineHealth {
  available: boolean;
  path: string;
  version: string | null;
  detail: string | null;
}

export interface ModelInfo {
  model_id: string;
  filename: string;
  format: "stl" | "obj";
  encoding: "binary" | "ascii";
  size_bytes: number;
  triangle_count: number | null;
  vertex_count: number | null;
  face_count: number | null;
}

export interface GridMetadata {
  grid: {
    resolution: Resolution;
    domain: Domain;
    spacing: { dx: number; dy: number; dz: number };
    cell_count: number;
    cell_volume: number;
    index_order: string;
  };
  classification: {
    fluid_count: number;
    solid_count: number;
    solid_rule: string;
    encoding?: "base64-uint8";
    data?: string;
  };
  test_box: TestBox | null;
  geometry_source?: "procedural_box" | "mesh" | "none";
  mesh?: MeshResult;
  voxelization?: VoxelizationStats;
  warnings: string[];
}

export interface MeshResult {
  input_triangles: number;
  triangles: number;
  degenerate_removed: number;
  watertight: boolean;
  consistently_oriented: boolean;
  bounds: { min: [number, number, number]; max: [number, number, number] };
  inside_domain: boolean;
  volume: number | null;
  voxel_volume: number;
  volume_relative_error: number | null;
}

export interface VoxelizationStats {
  triangles: number;
  cells_classified: number;
  rays_cast: number;
  ray_triangle_tests: number;
  ambiguous_crossings: number;
  fallback_cells: number;
  surface_cells: number;
  unresolved_cells: number;
  bvh_nodes: number;
  bvh_build_ms: number;
  classify_ms: number;
}

export interface GridJobRequest {
  domain: Domain;
  resolution: Resolution;
  geometry_source: "procedural_box" | "mesh";
  test_box: TestBox | null;
  model_id?: string | null;
  /** World placement of the uploaded model; requires model_id. */
  model_transform?: SerializedTransform | null;
  include_cells?: boolean;
  /** Numerical fields to export (opt-in), e.g. ["divergence_diagnostic"]. */
  fields?: FieldId[];
}

export type FieldId = "divergence_diagnostic";

export interface GridJob {
  job_id: string;
  status: "completed" | "failed";
  result: GridMetadata | null;
  error: { code: string; message: string } | null;
  /** Validated AFFIELD headers; data via fetchField(). */
  fields: FieldMetadata[];
}

export class ApiError extends Error {}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  let resp: Response;
  try {
    resp = await fetch(path, init);
  } catch {
    throw new ApiError("Cannot reach the AeroFlow backend. Is it running on port 8000?");
  }
  const body = await resp.json().catch(() => null);
  if (!resp.ok) {
    let message = `Request failed (${resp.status})`;
    if (body?.errors?.length) {
      message = body.errors
        .map((e: { field: string; message: string }) => `${e.field}: ${e.message}`)
        .join("; ");
    } else if (typeof body?.detail === "string") {
      message = body.detail;
    }
    throw new ApiError(message);
  }
  return body as T;
}

export function getHealth(): Promise<{ status: "ok"; engine: EngineHealth }> {
  return request("/api/health");
}

export interface Limits {
  max_cells: number;
  max_cells_per_axis: number;
  max_upload_mb: number;
  max_triangles: number;
}

export function getLimits(): Promise<Limits> {
  return request("/api/limits");
}

export function uploadModel(file: File): Promise<ModelInfo> {
  const form = new FormData();
  form.append("file", file);
  return request("/api/models", { method: "POST", body: form });
}

/** Raw AFFIELD bytes for one exported field (decode with decodeField). */
export async function fetchField(jobId: string, name: string, frame = 0): Promise<ArrayBuffer> {
  let resp: Response;
  try {
    resp = await fetch(`/api/grid-jobs/${encodeURIComponent(jobId)}/fields/${encodeURIComponent(name)}?frame=${frame}`);
  } catch {
    throw new ApiError("Cannot reach the AeroFlow backend. Is it running on port 8000?");
  }
  if (!resp.ok) {
    const body = await resp.json().catch(() => null);
    throw new ApiError(typeof body?.detail === "string" ? body.detail : `Field request failed (${resp.status})`);
  }
  return resp.arrayBuffer();
}

export function createGridJob(req: GridJobRequest): Promise<GridJob> {
  return request("/api/grid-jobs", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(req),
  });
}
