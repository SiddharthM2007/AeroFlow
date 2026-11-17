import { defaultRange, type ColorRange, type ColormapKind } from "./colormap";
import type { FieldMetadata } from "./format";

/**
 * Colour-range modes (Milestone 8):
 *   frame:  this frame's own range (the previous "auto" behaviour);
 *   all:    one range for every frame of the field, so colours mean the same value in every frame
 *           (computed from the per-frame header statistics; no extra downloads);
 *   manual: user-entered.
 */
export type RangeMode = "frame" | "all" | "manual";

/**
 * Frame 0 of a simulation pressure field is the impulse pressure of the initial projection
 * (rho phi / dt of the projection of the prescribed initial field), not a pressure of the flow.
 */
export function isInitialProjectionImpulse(meta: FieldMetadata): boolean {
  return meta.provenance.kind === "simulation" && meta.provenance.step === 0 && meta.name === "sim_pressure";
}

/** Frames that define the all-frames range (the initial-projection impulse is left out). */
export function rangeFrames(frames: FieldMetadata[]): FieldMetadata[] {
  const kept = frames.filter((f) => !isInitialProjectionImpulse(f));
  return kept.length > 0 ? kept : frames;
}

/** Union of the per-frame default ranges: symmetric for diverging maps, [min, max] otherwise. */
export function allFramesRange(frames: FieldMetadata[], kind: ColormapKind): ColorRange | null {
  const used = rangeFrames(frames);
  if (used.length === 0) return null;
  let min = Infinity;
  let max = -Infinity;
  for (const f of used) {
    const r = defaultRange(f, kind);
    if (Number.isFinite(r.min)) min = Math.min(min, r.min);
    if (Number.isFinite(r.max)) max = Math.max(max, r.max);
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return null;
  if (kind === "diverging") {
    const m = Math.max(Math.abs(min), Math.abs(max));
    return { min: -m, max: m };
  }
  return { min, max };
}
