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
  /** Present when projection fields were requested. */
  projection?: ProjectionStats | null;
  /** Present for Navier-Stokes runs. */
  simulation?: SimulationSummary | null;
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
  projection?: ProjectionRequest | null;
  simulation?: SimulationRequest | null;
}

export type FieldId =
  | "divergence_diagnostic"
  | "pressure"
  | "divergence_before"
  | "divergence_after"
  | "velocity_magnitude"
  | "sim_pressure"
  | "sim_velocity_magnitude"
  | "sim_divergence"
  | "sim_u"
  | "sim_v"
  | "sim_w";

export const PROJECTION_FIELD_IDS: FieldId[] = ["pressure", "divergence_before", "divergence_after", "velocity_magnitude"];
export const SIMULATION_FIELD_IDS: FieldId[] = [
  "sim_pressure",
  "sim_velocity_magnitude",
  "sim_divergence",
  "sim_u",
  "sim_v",
  "sim_w",
];

/**
 * Pressure Poisson solver (Milestone 7). "auto" picks MG-PCG from 1,000 fluid cells, else Jacobi-PCG.
 * Both solve the same equations to the same tolerance; MG-PCG only accelerates the solve.
 */
export type PressureSolver = "auto" | "mgpcg" | "jacobi_pcg";
export type ConcreteSolver = Exclude<PressureSolver, "auto">;

/** Solver diagnostics of one projection (engine `pressure_solver` block). */
export interface ProjectionSolverDiagnostics {
  requested: PressureSolver;
  selected: ConcreteSolver;
  used: ConcreteSolver;
  multigrid_levels: number;
  multigrid_setup_ms: number;
  pcg_ms: number;
  fallback: boolean;
  fallback_reason: string | null;
}

/** Solver diagnostics of a simulation run (engine `pressure_solver` block). */
export interface SimulationSolverDiagnostics {
  requested: PressureSolver;
  selected: ConcreteSolver;
  used: ConcreteSolver;
  pressure_iterations: number;
  multigrid_levels: number;
  multigrid_setups: number;
  multigrid_setup_ms: number;
  multigrid_memory_bytes: number;
  solve_seconds: number;
  fallbacks: number;
  fallback_reason: string | null;
}

/** Navier-Stokes run settings (Milestone 6); dt null = adaptive CFL step. */
export interface SimulationRequest {
  inlet_velocity: number;
  density: number;
  viscosity: number;
  duration: number;
  frame_interval: number;
  cfl: number;
  dt: number | null;
  max_steps: number;
  initial: "uniform" | "rest";
  pressure_solver: PressureSolver;
}

export interface SimulationEstimate {
  cells: number;
  dt: number;
  steps: number;
  frames: number;
  iterations_per_step: number;
  seconds: number;
  output_bytes: number;
  memory_bytes: number;
  /** Solver the estimate assumes ("auto" resolved by grid size), and the other solver for comparison. */
  solver?: ConcreteSolver;
  solver_requested?: PressureSolver;
  pressure_step_seconds?: number;
  setup_seconds?: number;
  alternative_solver?: ConcreteSolver;
  alternative_seconds?: number;
  multigrid_bytes?: number;
}

export interface SimulationProgress {
  step: number;
  time: number;
  duration: number;
  dt?: number;
  iterations?: number;
  relative_residual?: number;
  div_after_max?: number;
  max_speed?: number;
  frames?: number;
}

export interface SimulationSummary {
  status: "completed" | "stopped";
  stopped_reason: string | null;
  steps: number;
  time: number;
  frames: number;
  frame_times: number[];
  wall_seconds: number;
  dt_reductions: number;
  memory_bytes: number;
  mean_iterations: number;
  max_iterations: number;
  scheme: string;
  /** Per-step columns; numeric except "solver". */
  history: Record<string, (number | string)[]>;
  pressure_solver?: SimulationSolverDiagnostics;
}

/** Pressure projection settings (Milestone 5); dt null = CFL default 0.5 min(h) / U_in. */
export interface ProjectionRequest {
  inlet_velocity: number;
  density: number;
  dt: number | null;
  tolerance: number;
  max_iterations: number;
  pressure_solver: PressureSolver;
}

interface DivergenceSummary {
  fluid_cells: number;
  max_abs: number;
  mean_abs: number;
  rms: number;
}

export interface ProjectionStats {
  inlet_velocity: number;
  density: number;
  dt: number;
  dt_source: string;
  tolerance: number;
  iterations: number;
  relative_residual: number;
  converged: boolean;
  components: number;
  enclosed_components: number;
  divergence_before: DivergenceSummary | null;
  divergence_after: DivergenceSummary | null;
  fluxes: { inlet: number; outlet: number; wall_leak: number; inlet_area: number; outlet_area: number };
  memory_bytes: number;
  solve_ms: number;
  total_ms: number;
  pressure_solver?: ProjectionSolverDiagnostics;
}

export type JobStatus = "queued" | "running" | "completed" | "stopped" | "failed" | "cancelled";
export const FINAL_JOB_STATUSES: JobStatus[] = ["completed", "stopped", "failed", "cancelled"];

export interface GridJob {
  job_id: string;
  status: JobStatus;
  result: GridMetadata | null;
  error: { code: string; message: string } | null;
  /** Validated AFFIELD headers (one per field and frame); data via fetchField(). */
  fields: FieldMetadata[];
  estimate?: SimulationEstimate | null;
  progress?: SimulationProgress | null;
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

export function getGridJob(jobId: string): Promise<GridJob> {
  return request(`/api/grid-jobs/${encodeURIComponent(jobId)}`);
}

export function cancelGridJob(jobId: string): Promise<GridJob> {
  return request(`/api/grid-jobs/${encodeURIComponent(jobId)}/cancel`, { method: "POST" });
}

export interface EstimateResponse {
  estimate: SimulationEstimate;
  ok: boolean;
  problems: string[];
  budget: { seconds: number; max_field_bytes: number };
}

export function estimateGridJob(req: GridJobRequest): Promise<EstimateResponse> {
  return request("/api/grid-jobs/estimate", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(req),
  });
}

export function createGridJob(req: GridJobRequest): Promise<GridJob> {
  return request("/api/grid-jobs", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(req),
  });
}
