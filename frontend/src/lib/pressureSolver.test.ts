import { describe, expect, it } from "vitest";
import { SOLVER_LABELS, SOLVER_NOTE, SOLVER_OPTIONS, fallbackNotice, parsePressureSolver, solverHeadline } from "./pressureSolver";

describe("pressure solver selection", () => {
  it("offers Auto first and all three solvers", () => {
    expect(SOLVER_OPTIONS.map((o) => o.value)).toEqual(["auto", "mgpcg", "jacobi_pcg"]);
    expect(SOLVER_LABELS.mgpcg).toBe("MG-PCG");
    expect(SOLVER_NOTE).toMatch(/does not|unchanged/);
  });

  it("parses only known solver names", () => {
    expect(parsePressureSolver("auto")).toBe("auto");
    expect(parsePressureSolver("mgpcg")).toBe("mgpcg");
    expect(parsePressureSolver("jacobi_pcg")).toBe("jacobi_pcg");
    expect(parsePressureSolver("multigrid")).toBeNull();
    expect(parsePressureSolver(undefined)).toBeNull();
  });
});

describe("solver diagnostics", () => {
  it("names the solver that actually ran", () => {
    expect(solverHeadline({ requested: "mgpcg", selected: "mgpcg", used: "mgpcg" })).toBe("MG-PCG");
    expect(solverHeadline({ requested: "auto", selected: "mgpcg", used: "mgpcg" })).toBe("MG-PCG (selected by Auto)");
    expect(solverHeadline({ requested: "auto", selected: "jacobi_pcg", used: "jacobi_pcg" })).toBe(
      "Jacobi-PCG (selected by Auto)",
    );
    expect(solverHeadline({ requested: "mgpcg", selected: "mgpcg", used: "jacobi_pcg" })).toBe(
      "Jacobi-PCG (fallback from MG-PCG)",
    );
  });

  it("reports fallbacks, never hides them", () => {
    expect(fallbackNotice(0, null)).toBeNull();
    expect(fallbackNotice(1, "multigrid setup failed: x")).toBe(
      "Multigrid fell back to Jacobi-PCG in 1 pressure solve: multigrid setup failed: x.",
    );
    expect(fallbackNotice(3, null, 41)).toBe("Multigrid fell back to Jacobi-PCG in 3 of 41 pressure solves.");
  });
});
