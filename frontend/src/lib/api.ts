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
  warnings: string[];
}

export interface GridJobRequest {
  domain: Domain;
  resolution: Resolution;
  test_box: TestBox | null;
  model_id?: string | null;
  include_cells?: boolean;
}

export interface GridJob {
  job_id: string;
  status: "completed" | "failed";
  result: GridMetadata | null;
  error: { code: string; message: string } | null;
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

export function uploadModel(file: File): Promise<ModelInfo> {
  const form = new FormData();
  form.append("file", file);
  return request("/api/models", { method: "POST", body: form });
}

export function createGridJob(req: GridJobRequest): Promise<GridJob> {
  return request("/api/grid-jobs", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(req),
  });
}
