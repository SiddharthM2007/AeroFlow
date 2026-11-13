import type { ColorRange, ColormapKind } from "../lib/fields/colormap";
import type { FieldId, ProjectionStats, SimulationSummary } from "../lib/api";
import { SOLVER_LABELS, fallbackNotice, solverHeadline } from "../lib/pressureSolver";
import type { FieldMetadata, ScalarFieldData } from "../lib/fields/format";
import { provenanceLabel } from "../lib/fields/provenance";
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
  /** Frame index of the selected field (time-dependent fields have several). */
  frame: number;
  onFrameChange: (frame: number) => void;
  data: ScalarFieldData | null;
  loading: boolean;
  error: string | null;
  view: FieldViewSettings;
  onViewChange: (view: FieldViewSettings) => void;
  colormap: ColormapKind | null;
  range: ColorRange | null;
  planeCoordinate: number | null;
  projection: ProjectionStats | null;
  simulation: SimulationSummary | null;
}

const fmt = (v: number | null | undefined) =>
  v === null || v === undefined ? "–" : v === 0 ? "0" : Math.abs(v) >= 1e4 || Math.abs(v) < 1e-3 ? v.toExponential(3) : String(Number(v.toPrecision(5)));

export interface LastJobOutcome {
  status: "completed" | "stopped" | "failed" | "cancelled";
  requested: FieldId[];
  error: string | null;
}

/** Explains why no fields are shown for the most recent job instead of hiding the panel. */
export function FieldsNotice({ outcome, showingPreviousFields }: { outcome: LastJobOutcome; showingPreviousFields: boolean }) {
  let message: string;
  if (outcome.status === "failed") {
    message = `The last grid job failed, so no fields were exported: ${outcome.error ?? "unknown error"}`;
  } else if (outcome.status === "cancelled") {
    message = "The last simulation was cancelled; its frames were discarded.";
  } else if (outcome.status === "stopped") {
    message = `The simulation stopped early (${outcome.error ?? "budget reached"}). The frames written before the stop are shown below.`;
  } else if (outcome.requested.length === 0) {
    message = "No fields were exported for this job: none were ticked under Generate → Export fields. Tick the fields you want and generate again.";
  } else {
    message = `The job completed but returned none of the requested fields (${outcome.requested.join(", ")}). Check the backend log for this job.`;
  }
  return (
    <Panel title="Fields">
      <p className={outcome.status === "failed" ? "error" : "warn"} role="status">
        {message}
      </p>
      {showingPreviousFields && <p className="muted small">The fields below are from the previous successful job.</p>}
    </Panel>
  );
}

function ProjectionSummary({ stats }: { stats: ProjectionStats }) {
  const before = stats.divergence_before;
  const after = stats.divergence_after;
  const ps = stats.pressure_solver;
  const fallback = ps?.fallback ? fallbackNotice(1, ps.fallback_reason) : null;
  return (
    <div className="stats small projection-summary">
      {ps && (
        <div className="solver-summary">
          Pressure solver <b>{solverHeadline(ps)}</b> · PCG {fmt(ps.pcg_ms)} ms
          {ps.multigrid_levels > 0 && (
            <>
              {" "}
              · {ps.multigrid_levels} multigrid levels, setup {fmt(ps.multigrid_setup_ms)} ms
            </>
          )}
        </div>
      )}
      {fallback && (
        <p className="warn" role="status">
          {fallback}
        </p>
      )}
      <div>
        Pressure solve: <b>{stats.converged ? "converged" : "NOT converged"}</b> in <b>{stats.iterations}</b> PCG
        iterations · relative residual {fmt(stats.relative_residual)} (tol {fmt(stats.tolerance)}) · {fmt(stats.solve_ms)} ms
      </div>
      {before && after && (
        <div>
          Divergence max |div| {fmt(before.max_abs)} → <b>{fmt(after.max_abs)}</b> 1/s · RMS {fmt(before.rms)} →{" "}
          <b>{fmt(after.rms)}</b> 1/s
        </div>
      )}
      <div>
        Flux in {fmt(stats.fluxes.inlet)} · out {fmt(stats.fluxes.outlet)} m³/s · wall leak {fmt(stats.fluxes.wall_leak)} ·
        ρ {fmt(stats.density)} kg/m³ · Δt {fmt(stats.dt)} s ({stats.dt_source === "explicit" ? "set" : "CFL 0.5"})
      </div>
      {stats.enclosed_components > 0 && (
        <div className="muted">{stats.enclosed_components} enclosed fluid region(s) solved with a zero-mean pressure gauge</div>
      )}
    </div>
  );
}

const numeric = (column: (number | string)[] | undefined): number[] =>
  (column ?? []).filter((v): v is number => typeof v === "number");

function SimulationSummaryView({ sim }: { sim: SimulationSummary }) {
  const div = numeric(sim.history.div_after_max);
  const leak = numeric(sim.history.wall_leak);
  const inflow = numeric(sim.history.inflow);
  const outflow = numeric(sim.history.outflow);
  const last = (a: number[]) => (a.length ? a[a.length - 1] : null);
  const maxAbs = (a: number[]) => (a.length ? Math.max(...a.map(Math.abs)) : null);
  const ps = sim.pressure_solver;
  const fallback = ps ? fallbackNotice(ps.fallbacks, ps.fallback_reason, sim.steps + 1) : null;
  return (
    <div className="stats small projection-summary">
      <div>
        Run <b>{sim.status === "completed" ? "completed" : `stopped (${sim.stopped_reason ?? "budget"})`}</b>: {sim.steps} steps to t ={" "}
        <b>{fmt(sim.time)} s</b> · {sim.frames} frames · <b>{fmt(sim.wall_seconds)} s</b> wall
      </div>
      {ps && (
        <div className="solver-summary">
          Pressure solver <b>{solverHeadline(ps)}</b>
          {ps.requested !== "auto" && ps.requested !== ps.used && <> (requested {SOLVER_LABELS[ps.requested]})</>} ·
          pressure solve <b>{fmt(ps.solve_seconds)} s</b>
          {sim.wall_seconds > 0 && <> ({fmt((100 * ps.solve_seconds) / sim.wall_seconds)}% of wall time)</>} ·{" "}
          {ps.pressure_iterations.toLocaleString()} PCG iterations
          {ps.multigrid_levels > 0 && (
            <>
              {" "}
              · {ps.multigrid_levels} multigrid levels, setup {fmt(ps.multigrid_setup_ms)} ms, hierarchy{" "}
              {fmt(ps.multigrid_memory_bytes / 1048576)} MB
            </>
          )}
        </div>
      )}
      {fallback && (
        <p className="warn" role="status">
          {fallback}
        </p>
      )}
      <div>
        PCG iterations/step mean <b>{fmt(sim.mean_iterations)}</b> · max {sim.max_iterations} · max |div| after projection{" "}
        <b>{fmt(maxAbs(div))}</b> 1/s
      </div>
      <div>
        Flux in {fmt(last(inflow))} · out {fmt(last(outflow))} m³/s · max wall leak {fmt(maxAbs(leak))} · memory{" "}
        {fmt(sim.memory_bytes / 1048576)} MB{sim.dt_reductions > 0 ? ` · ${sim.dt_reductions} Δt reductions` : ""}
      </div>
    </div>
  );
}

/** Generic field selection and slice controls; nothing here is specific to one quantity. */
export function FieldsPanel(props: FieldsPanelProps) {
  const { fields, selected, data, view, onViewChange } = props;
  // One entry per field name; time-dependent fields carry one header per frame.
  const names = fields.filter((f, i) => fields.findIndex((g) => g.name === f.name) === i);
  const frames = fields.filter((f) => f.name === selected).sort((a, b) => a.frame.index - b.frame.index);
  const frameMeta = frames.find((f) => f.frame.index === props.frame) ?? frames[0] ?? null;
  const meta = data?.meta ?? frameMeta;
  const set = (patch: Partial<FieldViewSettings>) => onViewChange({ ...view, ...patch });
  const maxIndex = meta ? meta.shape[view.axis] - 1 : 0;

  return (
    <Panel title="Fields">
      <label className="field">
        <span>Field</span>
        <select value={selected ?? ""} onChange={(e) => props.onSelect(e.target.value)}>
          {names.map((f) => (
            <option key={f.name} value={f.name}>
              {f.long_name} [{f.units}]
              {fields.filter((g) => g.name === f.name).length > 1 ? " — time series" : f.frame.label ? ` — ${f.frame.label}` : ""}
            </option>
          ))}
        </select>
      </label>
      {frames.length > 1 && (
        <div className="frame-select">
          <div className="row">
            <span className="small">Frame</span>
            <input
              type="range"
              min={0}
              max={frames.length - 1}
              value={Math.max(0, frames.indexOf(frameMeta!))}
              onChange={(e) => props.onFrameChange(frames[Number(e.target.value)].frame.index)}
            />
            <span className="mono">
              {frames.indexOf(frameMeta!) + 1}/{frames.length}
            </span>
          </div>
          <p className="small">
            Simulation time <b>t = {fmt(frameMeta?.frame.time_s ?? null)} s</b>
            {typeof frameMeta?.provenance.step === "number" && <> · step {frameMeta.provenance.step as number}</>}
          </p>
        </div>
      )}

      {meta && provenanceLabel(meta) && (
        <p className="diagnostic-note">
          <b>{provenanceLabel(meta)!.note}</b> {meta.provenance.description}
        </p>
      )}
      {props.projection && meta?.provenance.kind !== "simulation" && <ProjectionSummary stats={props.projection} />}
      {props.simulation && meta?.provenance.kind === "simulation" && <SimulationSummaryView sim={props.simulation} />}
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
