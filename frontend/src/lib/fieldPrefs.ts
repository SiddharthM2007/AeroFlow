// Per-viewer convenience: remember which fields to export and the projection settings across
// reloads (browser storage only; never required for correctness, so every access is guarded).
import { parsePressureSolver } from "./pressureSolver";
import {
  PROJECTION_FIELD_IDS,
  SIMULATION_FIELD_IDS,
  type FieldId,
  type ProjectionRequest,
  type SimulationRequest,
} from "./api";

const KEY = "aeroflow.fieldExport.v1";
const KNOWN: FieldId[] = ["divergence_diagnostic", ...PROJECTION_FIELD_IDS, ...SIMULATION_FIELD_IDS];

export interface FieldPrefs {
  fields: FieldId[];
  projection: ProjectionRequest;
  simulation: SimulationRequest;
}

export const DEFAULT_SIMULATION: SimulationRequest = {
  inlet_velocity: 5,
  density: 1.225,
  viscosity: 1.5e-5,
  duration: 0.5,
  frame_interval: 0.05,
  cfl: 0.5,
  dt: null,
  max_steps: 2000,
  initial: "uniform",
  pressure_solver: "auto",
};

export const DEFAULT_PROJECTION: ProjectionRequest = {
  inlet_velocity: 5,
  density: 1.225,
  dt: null,
  tolerance: 1e-10,
  max_iterations: 10000,
  pressure_solver: "auto",
};

type KeyValueStore = Pick<Storage, "getItem" | "setItem">;

export function browserStorage(): KeyValueStore | undefined {
  try {
    return typeof window !== "undefined" ? window.localStorage : undefined;
  } catch {
    return undefined; // blocked storage (private mode, sandboxed previews)
  }
}

const positive = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v > 0;

/** Stored preferences, validated field by field; anything malformed falls back to the defaults. */
export function loadFieldPrefs(storage: KeyValueStore | undefined): FieldPrefs {
  const defaults: FieldPrefs = { fields: [], projection: { ...DEFAULT_PROJECTION }, simulation: { ...DEFAULT_SIMULATION } };
  let raw: unknown;
  try {
    const text = storage?.getItem(KEY);
    if (!text) return defaults;
    raw = JSON.parse(text);
  } catch {
    return defaults;
  }
  if (!raw || typeof raw !== "object") return defaults;
  const r = raw as { fields?: unknown; projection?: Record<string, unknown>; simulation?: Record<string, unknown> };
  const fields = Array.isArray(r.fields)
    ? KNOWN.filter((id) => (r.fields as unknown[]).includes(id))
    : defaults.fields;
  const p = r.projection ?? {};
  const projection: ProjectionRequest = {
    inlet_velocity:
      typeof p.inlet_velocity === "number" && Number.isFinite(p.inlet_velocity) && p.inlet_velocity >= 0
        ? p.inlet_velocity
        : DEFAULT_PROJECTION.inlet_velocity,
    density: positive(p.density) ? p.density : DEFAULT_PROJECTION.density,
    dt: positive(p.dt) ? p.dt : null,
    tolerance: positive(p.tolerance) && p.tolerance < 1 ? p.tolerance : DEFAULT_PROJECTION.tolerance,
    max_iterations:
      Number.isInteger(p.max_iterations) && (p.max_iterations as number) >= 1
        ? (p.max_iterations as number)
        : DEFAULT_PROJECTION.max_iterations,
    pressure_solver: parsePressureSolver(p.pressure_solver) ?? DEFAULT_PROJECTION.pressure_solver,
  };
  const q = r.simulation ?? {};
  const nonNegative = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v >= 0;
  const simulation: SimulationRequest = {
    inlet_velocity: nonNegative(q.inlet_velocity) ? q.inlet_velocity : DEFAULT_SIMULATION.inlet_velocity,
    density: positive(q.density) ? q.density : DEFAULT_SIMULATION.density,
    viscosity: nonNegative(q.viscosity) ? q.viscosity : DEFAULT_SIMULATION.viscosity,
    duration: positive(q.duration) ? q.duration : DEFAULT_SIMULATION.duration,
    frame_interval: positive(q.frame_interval) ? q.frame_interval : DEFAULT_SIMULATION.frame_interval,
    cfl: positive(q.cfl) && q.cfl <= 1 ? q.cfl : DEFAULT_SIMULATION.cfl,
    dt: positive(q.dt) ? q.dt : null,
    max_steps:
      Number.isInteger(q.max_steps) && (q.max_steps as number) >= 1 ? (q.max_steps as number) : DEFAULT_SIMULATION.max_steps,
    initial: q.initial === "rest" ? "rest" : "uniform",
    pressure_solver: parsePressureSolver(q.pressure_solver) ?? DEFAULT_SIMULATION.pressure_solver,
  };
  return { fields, projection, simulation };
}

export function saveFieldPrefs(storage: KeyValueStore | undefined, prefs: FieldPrefs): void {
  try {
    storage?.setItem(KEY, JSON.stringify(prefs));
  } catch {
    // Quota or blocked storage: preferences are optional.
  }
}
