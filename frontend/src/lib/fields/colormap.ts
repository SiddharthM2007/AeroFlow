// Scalar normalization and color maps for field rendering (independent of any specific field).
import type { FieldMetadata } from "./format";
import type { ScalarSlice } from "./sampling";

export type ColormapKind = "diverging" | "sequential";

export interface ColorRange {
  min: number;
  max: number;
}

type RGB = [number, number, number];

// Blue - light grey - red (cool/warm), symmetric about its midpoint.
const DIVERGING: RGB[] = [
  [59, 76, 192],
  [141, 176, 254],
  [221, 221, 221],
  [244, 154, 123],
  [180, 4, 38],
];

// Perceptually uniform sequential (viridis key colors).
const SEQUENTIAL: RGB[] = [
  [68, 1, 84],
  [59, 82, 139],
  [33, 145, 140],
  [94, 201, 98],
  [253, 231, 37],
];

const STOPS: Record<ColormapKind, RGB[]> = { diverging: DIVERGING, sequential: SEQUENTIAL };

/** Signed quantities use a diverging map centered at zero; nonnegative ones a sequential map. */
export function defaultColormap(meta: FieldMetadata): ColormapKind {
  return meta.signed ? "diverging" : "sequential";
}

/** Symmetric [-max|v|, +max|v|] for diverging maps; [min, max] of valid samples otherwise. */
export function defaultRange(meta: FieldMetadata, kind: ColormapKind): ColorRange {
  const s = meta.stats;
  if (kind === "diverging") {
    const m = s.max_abs ?? 0;
    return { min: -m, max: m };
  }
  return { min: s.min ?? 0, max: s.max ?? 0 };
}

/** Maps a value to [0, 1]; a degenerate range maps to the neutral point (0.5 diverging, 0 sequential). */
export function normalize(value: number, range: ColorRange, kind: ColormapKind): number {
  const span = range.max - range.min;
  if (!(span > 0) || !Number.isFinite(span)) return kind === "diverging" ? 0.5 : 0;
  const t = (value - range.min) / span;
  return t <= 0 ? 0 : t >= 1 ? 1 : t;
}

export function colormapRGB(kind: ColormapKind, t: number): RGB {
  const stops = STOPS[kind];
  const x = Math.min(Math.max(t, 0), 1) * (stops.length - 1);
  const i = Math.min(Math.floor(x), stops.length - 2);
  const f = x - i;
  const a = stops[i];
  const b = stops[i + 1];
  return [0, 1, 2].map((c) => Math.round(a[c] + (b[c] - a[c]) * f)) as RGB;
}

/** RGBA8 texels for a slice; invalid samples are fully transparent. */
export function sliceToRGBA(slice: ScalarSlice, range: ColorRange, kind: ColormapKind): Uint8Array {
  const out = new Uint8Array(slice.width * slice.height * 4);
  for (let n = 0; n < slice.values.length; n++) {
    if (!slice.valid[n] || !Number.isFinite(slice.values[n])) continue; // stays [0, 0, 0, 0]
    const [r, g, b] = colormapRGB(kind, normalize(slice.values[n], range, kind));
    out[4 * n] = r;
    out[4 * n + 1] = g;
    out[4 * n + 2] = b;
    out[4 * n + 3] = 255;
  }
  return out;
}

export function cssGradient(kind: ColormapKind, steps = 16): string {
  const parts = Array.from({ length: steps + 1 }, (_, n) => {
    const [r, g, b] = colormapRGB(kind, n / steps);
    return `rgb(${r}, ${g}, ${b}) ${((100 * n) / steps).toFixed(1)}%`;
  });
  return `linear-gradient(to right, ${parts.join(", ")})`;
}

/**
 * Legend tick values: ends of the range, plus zero when it lies strictly inside. A degenerate
 * range (e.g. an exactly zero field, auto range [-0, 0]) yields a single tick; -0 is shown as 0.
 */
export function legendTicks(range: ColorRange): number[] {
  const lo = range.min === 0 ? 0 : range.min;
  const hi = range.max === 0 ? 0 : range.max;
  if (!(hi > lo)) return [lo];
  const ticks = [lo, hi];
  if (lo < 0 && hi > 0) ticks.splice(1, 0, 0);
  return ticks;
}
