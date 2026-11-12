import type { ColorRange, ColormapKind } from "../lib/fields/colormap";
import type { FieldMetadata, ScalarFieldData } from "../lib/fields/format";
import { AXIS_NAMES, type Axis } from "../lib/fields/sampling";
import { NumberField, Panel } from "./Fields";

export interface FieldViewSettings {
  visible: boolean;
  axis: Axis;
  index: number;
  rangeMode: "auto" | "manual";
  manual: ColorRange;
  opacity: number;
}

interface FieldsPanelProps {
  fields: FieldMetadata[];
  selected: string | null;
  onSelect: (name: string) => void;
  data: ScalarFieldData | null;
  loading: boolean;
  error: string | null;
  view: FieldViewSettings;
  onViewChange: (view: FieldViewSettings) => void;
  colormap: ColormapKind | null;
  range: ColorRange | null;
  planeCoordinate: number | null;
}

const fmt = (v: number | null | undefined) =>
  v === null || v === undefined ? "–" : v === 0 ? "0" : Math.abs(v) >= 1e4 || Math.abs(v) < 1e-3 ? v.toExponential(3) : String(Number(v.toPrecision(5)));

/** Generic field selection and slice controls; nothing here is specific to one quantity. */
export function FieldsPanel(props: FieldsPanelProps) {
  const { fields, selected, data, view, onViewChange } = props;
  const meta = data?.meta ?? fields.find((f) => f.name === selected) ?? null;
  const set = (patch: Partial<FieldViewSettings>) => onViewChange({ ...view, ...patch });
  const maxIndex = meta ? meta.shape[view.axis] - 1 : 0;

  return (
    <Panel title="Fields">
      <label className="field">
        <span>Field</span>
        <select value={selected ?? ""} onChange={(e) => props.onSelect(e.target.value)}>
          {fields.map((f) => (
            <option key={`${f.name}:${f.frame.index}`} value={f.name}>
              {f.long_name} [{f.units}]{f.frame.label ? ` — ${f.frame.label}` : ""}
            </option>
          ))}
        </select>
      </label>

      {meta?.provenance.kind === "diagnostic" && (
        <p className="diagnostic-note">
          <b>Diagnostic, not a solved airflow.</b> {meta.provenance.description}
        </p>
      )}
      {props.loading && <p className="muted small">Loading field data…</p>}
      {props.error && <p className="error">{props.error}</p>}

      {meta && (
        <div className="stats small">
          <div>
            Max |value| <b>{fmt(meta.stats.max_abs)}</b> {meta.units} · RMS <b>{fmt(meta.stats.rms)}</b> {meta.units}
          </div>
          <div>
            Mean |value| {fmt(meta.stats.mean_abs)} · range [{fmt(meta.stats.min)}, {fmt(meta.stats.max)}] {meta.units}
          </div>
          <div>
            Valid samples <b>{meta.stats.valid_count.toLocaleString()}</b> of{" "}
            {(meta.shape[0] * meta.shape[1] * meta.shape[2]).toLocaleString()} ({meta.location}, {meta.shape.join("×")})
          </div>
        </div>
      )}

      {data && (
        <>
          <label className="check">
            <input type="checkbox" checked={view.visible} onChange={(e) => set({ visible: e.target.checked })} />
            Show slice
          </label>
          <div className="segmented">
            {([0, 1, 2] as Axis[]).map((a) => (
              <button
                key={a}
                className={view.axis === a ? "active" : ""}
                onClick={() => set({ axis: a, index: Math.min(view.index, (meta?.shape[a] ?? 1) - 1) })}
              >
                {AXIS_NAMES[a].toUpperCase()} slice
              </button>
            ))}
          </div>
          <div className="row">
            <input
              type="range"
              min={0}
              max={maxIndex}
              value={Math.min(view.index, maxIndex)}
              onChange={(e) => set({ index: Number(e.target.value) })}
            />
            <span className="mono">{Math.min(view.index, maxIndex)}</span>
          </div>
          {props.planeCoordinate !== null && (
            <p className="muted small">
              {AXIS_NAMES[view.axis]} = {fmt(props.planeCoordinate)} m
            </p>
          )}
          <div className="segmented">
            {(["auto", "manual"] as const).map((m) => (
              <button
                key={m}
                className={view.rangeMode === m ? "active" : ""}
                onClick={() => set({ rangeMode: m, manual: m === "manual" && props.range ? props.range : view.manual })}
              >
                {m === "auto" ? (props.colormap === "diverging" ? "Auto (±max, 0-centred)" : "Auto (min–max)") : "Manual range"}
              </button>
            ))}
          </div>
          {view.rangeMode === "manual" && (
            <div className="vec3-inputs">
              <NumberField label={`min [${meta?.units}]`} value={view.manual.min} onChange={(min) => set({ manual: { ...view.manual, min } })} />
              <NumberField label={`max [${meta?.units}]`} value={view.manual.max} onChange={(max) => set({ manual: { ...view.manual, max } })} />
            </div>
          )}
          <label className="field">
            <span>Slice opacity</span>
            <input
              type="range"
              min={0.2}
              max={1}
              step={0.05}
              value={view.opacity}
              onChange={(e) => set({ opacity: Number(e.target.value) })}
            />
          </label>
        </>
      )}
    </Panel>
  );
}
