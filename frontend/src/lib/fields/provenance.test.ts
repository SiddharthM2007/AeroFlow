import { describe, expect, it } from "vitest";
import { buildGridJobRequest } from "../geometrySource";
import type { FieldMetadata } from "./format";
import { provenanceLabel } from "./provenance";

const meta = (provenance: FieldMetadata["provenance"]) => ({ provenance }) as FieldMetadata;

describe("provenanceLabel", () => {
  it("labels every unsolved field, by kind", () => {
    expect(provenanceLabel(meta({ kind: "diagnostic", solved: false }))?.badge).toMatch(/not a solved airflow/);
    expect(provenanceLabel(meta({ kind: "projection", solved: false }))?.badge).toMatch(/not a time-accurate flow/);
    expect(provenanceLabel(meta({ kind: "something_new", solved: false }))?.badge).toBe("Not a solved flow");
    expect(provenanceLabel(meta({ kind: "solution", solved: true }))).toBeNull();
    // Time-dependent runs are always labelled, so they cannot be confused with the diagnostic.
    expect(provenanceLabel(meta({ kind: "simulation", solved: true }))?.badge).toMatch(/Time-dependent Navier–Stokes/);
  });
});

describe("projection request", () => {
  const domain = { lx: 1, ly: 1, lz: 1 };
  const res = { nx: 4, ny: 4, nz: 4 };
  const settings = {
    inlet_velocity: 5,
    density: 1.225,
    dt: null,
    tolerance: 1e-10,
    max_iterations: 100,
    pressure_solver: "auto" as const,
  };

  it("sends projection settings only with fields that use them", () => {
    expect(buildGridJobRequest("procedural_box", domain, res, null, null, ["pressure"], settings).projection).toEqual(settings);
    expect(buildGridJobRequest("procedural_box", domain, res, null, null, [], settings).projection).toBeNull();
  });
});
