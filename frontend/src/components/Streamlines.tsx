import { useEffect, useMemo } from "react";
import { Float32BufferAttribute as F32 } from "three";
import { BufferAttribute, BufferGeometry, Float32BufferAttribute } from "three";
import { colormapRGB, normalize, type ColorRange } from "../lib/fields/colormap";
import type { PathlineResult, TraceResult } from "../lib/streamlines/engine";
import { pathsAtTime } from "../lib/streamlines/flowDisplay";

/** Instantaneous streamlines as 3D polylines, coloured by local |u| with the sequential map. */
export function Streamlines({ result, range }: { result: TraceResult; range: ColorRange }) {
  const geometry = useMemo(() => {
    const n = result.speeds.length;
    const colors = new Float32Array(3 * n);
    for (let p = 0; p < n; p++) {
      const [r, g, b] = colormapRGB("sequential", normalize(result.speeds[p], range, "sequential"));
      colors[3 * p] = r / 255;
      colors[3 * p + 1] = g / 255;
      colors[3 * p + 2] = b / 255;
    }
    // Segments only within each line (never joining consecutive lines).
    let segments = 0;
    for (let l = 0; l + 1 < result.offsets.length; l++) segments += Math.max(result.offsets[l + 1] - result.offsets[l] - 1, 0);
    const index = new Uint32Array(2 * segments);
    let s = 0;
    for (let l = 0; l + 1 < result.offsets.length; l++)
      for (let p = result.offsets[l]; p + 1 < result.offsets[l + 1]; p++) {
        index[s++] = p;
        index[s++] = p + 1;
      }
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(result.positions, 3));
    g.setAttribute("color", new Float32BufferAttribute(colors, 3));
    g.setIndex(new BufferAttribute(index, 1));
    return g;
  }, [result, range.min, range.max]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  return (
    <lineSegments geometry={geometry} renderOrder={3}>
      <lineBasicMaterial vertexColors transparent opacity={0.95} />
    </lineSegments>
  );
}

/** Range of the computed speeds (used when |u| was not exported as a field). */
export function speedRange(result: { speeds: Float32Array }): ColorRange {
  let min = Infinity;
  let max = -Infinity;
  for (const s of result.speeds) {
    if (s < min) min = s;
    if (s > max) max = s;
  }
  return Number.isFinite(min) ? { min, max } : { min: 0, max: 1 };
}

function speedColors(speeds: Float32Array, range: ColorRange): Float32Array {
  const colors = new Float32Array(3 * speeds.length);
  for (let p = 0; p < speeds.length; p++) {
    const [r, g, b] = colormapRGB("sequential", normalize(speeds[p], range, "sequential"));
    colors[3 * p] = r / 255;
    colors[3 * p + 1] = g / 255;
    colors[3 * p + 2] = b / 255;
  }
  return colors;
}

/**
 * Particle paths revealed up to the displayed simulation time `time`, with tracer markers at that
 * time. Vertices and colours are built once per result; only the segment index changes with time.
 */
export function Pathlines({ result, time, range, showMarkers }: { result: PathlineResult; time: number; range: ColorRange; showMarkers: boolean }) {
  const geometry = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(result.positions, 3));
    g.setAttribute("color", new F32(speedColors(result.speeds, range), 3));
    return g;
  }, [result, range.min, range.max]);
  const shown = useMemo(() => pathsAtTime(result, time), [result, time]);
  useEffect(() => {
    geometry.setIndex(new BufferAttribute(shown.index, 1));
  }, [geometry, shown]);
  const markers = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(shown.markers, 3));
    return g;
  }, [shown]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => () => markers.dispose(), [markers]);
  return (
    <>
      <lineSegments geometry={geometry} renderOrder={3}>
        <lineBasicMaterial vertexColors transparent opacity={0.9} />
      </lineSegments>
      {showMarkers && shown.alive > 0 && (
        <points geometry={markers} renderOrder={4}>
          <pointsMaterial color="#ff2bd6" size={6} sizeAttenuation={false} />
        </points>
      )}
    </>
  );
}
