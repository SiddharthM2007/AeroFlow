import { cssGradient, legendTicks, type ColorRange, type ColormapKind } from "../lib/fields/colormap";
import type { FieldMetadata } from "../lib/fields/format";
import { provenanceLabel } from "../lib/fields/provenance";
import { AXIS_NAMES, type Axis } from "../lib/fields/sampling";

interface FieldLegendProps {
  meta: FieldMetadata;
  range: ColorRange;
  colormap: ColormapKind;
  axis: Axis;
  index: number;
  planeCoordinate: number;
}

const fmt = (v: number) => (v === 0 ? "0" : Math.abs(v) >= 1e4 || Math.abs(v) < 1e-3 ? v.toExponential(2) : String(Number(v.toPrecision(4))));

export function FieldLegend({ meta, range, colormap, axis, index, planeCoordinate }: FieldLegendProps) {
  const ticks = legendTicks(range);
  const span = range.max - range.min;
  return (
    <div className="field-legend">
      {provenanceLabel(meta) && <div className="legend-badge">{provenanceLabel(meta)!.badge}</div>}
      <div className="legend-title">
        {meta.long_name} <span className="muted">[{meta.units}]</span>
      </div>
      <div className="legend-bar" style={{ background: cssGradient(colormap) }} />
      <div className="legend-ticks">
        {ticks.length === 1 ? (
          <span style={{ left: "50%", transform: "translateX(-50%)" }}>{fmt(ticks[0])}</span>
        ) : (
          ticks.map((t, n) => (
            <span key={n} style={{ left: `${((t - range.min) / span) * 100}%` }}>
              {fmt(t)}
            </span>
          ))
        )}
      </div>
      <div className="legend-meta muted">
        {AXIS_NAMES[axis]}-slice {index} at {AXIS_NAMES[axis]} = {fmt(planeCoordinate)} m · frame {meta.frame.index}
        {meta.frame.label ? ` (${meta.frame.label})` : ""}
      </div>
    </div>
  );
}
