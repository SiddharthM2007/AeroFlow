// Human-readable labels for a field's provenance. Generic: any field that is not a solved flow
// gets a visible badge, whatever produced it.
import type { FieldMetadata } from "./format";

export interface ProvenanceLabel {
  badge: string;
  note: string;
}

const KIND_LABELS: Record<string, ProvenanceLabel> = {
  diagnostic: { badge: "Diagnostic — not a solved airflow", note: "Diagnostic, not a solved airflow." },
  projection: {
    badge: "Single projection — not a time-accurate flow",
    note: "Single pressure projection, not a time-accurate flow solution.",
  },
  simulation: {
    badge: "Time-dependent Navier–Stokes (first-order, grid-dependent)",
    note: "Time-dependent incompressible Navier–Stokes solution.",
  },
};

/**
 * Badge and note for the UI. Unsolved fields always get one; time-dependent simulations get one
 * too so they are never mistaken for the single-projection diagnostic. Other solved fields: null.
 */
export function provenanceLabel(meta: FieldMetadata): ProvenanceLabel | null {
  if (meta.provenance.kind === "simulation") return KIND_LABELS.simulation;
  if (meta.provenance.solved) return null;
  return KIND_LABELS[meta.provenance.kind] ?? { badge: "Not a solved flow", note: "Not a solved flow." };
}
