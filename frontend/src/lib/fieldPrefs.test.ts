import { describe, expect, it } from "vitest";
import { DEFAULT_PROJECTION, DEFAULT_SIMULATION, loadFieldPrefs, saveFieldPrefs } from "./fieldPrefs";

function memory() {
  const data = new Map<string, string>();
  return { getItem: (k: string) => data.get(k) ?? null, setItem: (k: string, v: string) => void data.set(k, v), data };
}

describe("field export preferences", () => {
  it("round-trips selections and projection settings", () => {
    const s = memory();
    const prefs = {
      fields: ["pressure", "velocity_magnitude"] as const,
      projection: { ...DEFAULT_PROJECTION, inlet_velocity: 8, dt: 0.002 },
    };
    saveFieldPrefs(s, { fields: [...prefs.fields], projection: prefs.projection, simulation: DEFAULT_SIMULATION });
    expect(loadFieldPrefs(s)).toMatchObject({ fields: ["pressure", "velocity_magnitude"], projection: prefs.projection });
  });

  it("defaults when storage is missing, blocked, corrupt or partly invalid", () => {
    expect(loadFieldPrefs(undefined)).toMatchObject({ fields: [], projection: DEFAULT_PROJECTION });
    const throwing = {
      getItem: () => {
        throw new Error("blocked");
      },
      setItem: () => {
        throw new Error("blocked");
      },
    };
    expect(loadFieldPrefs(throwing).fields).toEqual([]);
    expect(() =>
      saveFieldPrefs(throwing, { fields: [], projection: DEFAULT_PROJECTION, simulation: DEFAULT_SIMULATION }),
    ).not.toThrow();
    const s = memory();
    s.setItem("aeroflow.fieldExport.v1", "{not json");
    expect(loadFieldPrefs(s).fields).toEqual([]);
    s.setItem(
      "aeroflow.fieldExport.v1",
      JSON.stringify({ fields: ["pressure", "vorticity", 3], projection: { density: -1, tolerance: 2, max_iterations: 2.5, dt: 0 } }),
    );
    expect(loadFieldPrefs(s)).toMatchObject({ fields: ["pressure"], projection: DEFAULT_PROJECTION });
  });
});

describe("simulation preferences", () => {
  it("round-trips and validates simulation settings", () => {
    const s = memory();
    const simulation = { ...DEFAULT_SIMULATION, duration: 1.5, viscosity: 0, dt: 0.001, initial: "rest" as const };
    saveFieldPrefs(s, { fields: ["sim_pressure", "sim_u"], projection: DEFAULT_PROJECTION, simulation });
    expect(loadFieldPrefs(s)).toEqual({ fields: ["sim_pressure", "sim_u"], projection: DEFAULT_PROJECTION, simulation });
    s.setItem("aeroflow.fieldExport.v1", JSON.stringify({ simulation: { cfl: 3, duration: -1, initial: "x", max_steps: 0 } }));
    expect(loadFieldPrefs(s).simulation).toEqual(DEFAULT_SIMULATION);
  });
});

describe("pressure solver preference", () => {
  it("defaults to auto, round-trips and rejects unknown values", () => {
    const s = memory();
    expect(loadFieldPrefs(s).simulation.pressure_solver).toBe("auto");
    expect(loadFieldPrefs(s).projection.pressure_solver).toBe("auto");
    saveFieldPrefs(s, {
      fields: ["sim_pressure"],
      projection: { ...DEFAULT_PROJECTION, pressure_solver: "jacobi_pcg" },
      simulation: { ...DEFAULT_SIMULATION, pressure_solver: "mgpcg" },
    });
    expect(loadFieldPrefs(s).simulation.pressure_solver).toBe("mgpcg");
    expect(loadFieldPrefs(s).projection.pressure_solver).toBe("jacobi_pcg");
    s.setItem(
      "aeroflow.fieldExport.v1",
      JSON.stringify({ simulation: { pressure_solver: "multigrid" }, projection: { pressure_solver: 3 } }),
    );
    expect(loadFieldPrefs(s).simulation.pressure_solver).toBe("auto");
    expect(loadFieldPrefs(s).projection.pressure_solver).toBe("auto");
  });
});
