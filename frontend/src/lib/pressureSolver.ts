import type { ConcreteSolver, PressureSolver } from "./api";

export const SOLVER_LABELS: Record<PressureSolver, string> = {
  auto: "Auto",
  mgpcg: "MG-PCG",
  jacobi_pcg: "Jacobi-PCG",
};

export const SOLVER_OPTIONS: { value: PressureSolver; label: string }[] = [
  { value: "auto", label: "Auto (recommended): MG-PCG from 1,000 cells" },
  { value: "mgpcg", label: "MG-PCG (multigrid-preconditioned CG)" },
  { value: "jacobi_pcg", label: "Jacobi-PCG (original solver)" },
];

/** Shown under the selector: what the choice does and does not change. */
export const SOLVER_NOTE =
  "MG-PCG only accelerates the pressure solve: it solves the same equations to the same tolerance, so the " +
  "flow physics and results are unchanged. If multigrid fails, the solve finishes with Jacobi-PCG and the " +
  "fallback is reported.";

export function parsePressureSolver(value: unknown): PressureSolver | null {
  return value === "auto" || value === "mgpcg" || value === "jacobi_pcg" ? value : null;
}

/** "MG-PCG", or "MG-PCG (selected by Auto)", or "Jacobi-PCG (fallback from MG-PCG)". */
export function solverHeadline(d: {
  requested: PressureSolver;
  selected: ConcreteSolver;
  used: ConcreteSolver;
}): string {
  const used = SOLVER_LABELS[d.used];
  if (d.used !== d.selected) return `${used} (fallback from ${SOLVER_LABELS[d.selected]})`;
  if (d.requested === "auto") return `${used} (selected by Auto)`;
  return used;
}

/** Fallback sentence for summaries, or null when the requested solver finished every solve. */
export function fallbackNotice(fallbacks: number, reason: string | null, total?: number): string | null {
  if (fallbacks <= 0) return null;
  const count = total ? `${fallbacks} of ${total} pressure solves` : `${fallbacks} pressure solve${fallbacks === 1 ? "" : "s"}`;
  return `Multigrid fell back to Jacobi-PCG in ${count}${reason ? `: ${reason}` : ""}.`;
}
