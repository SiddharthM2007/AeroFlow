import type { EstimateResponse, GridJob, PressureSolver, SimulationRequest } from "../lib/api";
import { SOLVER_LABELS, SOLVER_NOTE, SOLVER_OPTIONS, parsePressureSolver } from "../lib/pressureSolver";
import { NumberField } from "./Fields";

/** Pressure solver selector with an explanation of what the choice changes (speed, not physics). */
export function PressureSolverField({ value, onChange }: { value: PressureSolver; onChange: (v: PressureSolver) => void }) {
  return (
    <div className="solver-field">
      <label className="field">
        <span>Pressure solver</span>
        <select
          aria-label="Pressure solver"
          value={value}
          onChange={(e) => onChange(parsePressureSolver(e.target.value) ?? "auto")}
        >
          {SOLVER_OPTIONS.map((o) => (
            <option key={o.value} value={o.value}>
              {o.label}
            </option>
          ))}
        </select>
      </label>
      <p className="muted small">{SOLVER_NOTE}</p>
    </div>
  );
}

const fmt = (v: number | null | undefined) =>
  v === null || v === undefined ? "–" : v === 0 ? "0" : Math.abs(v) >= 1e4 || Math.abs(v) < 1e-3 ? v.toExponential(3) : String(Number(v.toPrecision(4)));

function fmtDuration(seconds: number): string {
  if (seconds < 60) return `${fmt(seconds)} s`;
  if (seconds < 3600) return `${fmt(seconds / 60)} min`;
  return `${fmt(seconds / 3600)} h`;
}

const fmtBytes = (b: number) => (b >= 1 << 30 ? `${fmt(b / (1 << 30))} GB` : `${fmt(b / (1 << 20))} MB`);

interface SettingsProps {
  settings: SimulationRequest;
  onChange: (settings: SimulationRequest) => void;
  estimate: EstimateResponse | null;
  estimateError: string | null;
}

/** Navier-Stokes run settings plus the backend's preflight estimate (runs over budget are rejected). */
export function SimulationSettingsView({ settings, onChange, estimate, estimateError }: SettingsProps) {
  const set = (patch: Partial<SimulationRequest>) => onChange({ ...settings, ...patch });
  const est = estimate?.estimate;
  return (
    <div className="projection-settings">
      <div className="vec3-inputs">
        <NumberField label="U_in (m/s)" value={settings.inlet_velocity} min={0} onChange={(inlet_velocity) => set({ inlet_velocity })} />
        <NumberField label="ρ (kg/m³)" value={settings.density} min={1e-9} onChange={(density) => set({ density })} />
        <NumberField label="ν (m²/s)" value={settings.viscosity} min={0} onChange={(viscosity) => set({ viscosity })} />
      </div>
      <div className="vec3-inputs">
        <NumberField label="duration (s)" value={settings.duration} min={1e-9} onChange={(duration) => set({ duration })} />
        <NumberField label="frame every (s)" value={settings.frame_interval} min={1e-9} onChange={(frame_interval) => set({ frame_interval })} />
        <NumberField label="CFL" value={settings.cfl} min={1e-6} max={1} onChange={(cfl) => set({ cfl })} />
      </div>
      <div className="vec3-inputs">
        <NumberField label="max steps" value={settings.max_steps} integer min={1} max={1000000} onChange={(max_steps) => set({ max_steps })} />
        <label className="field">
          <span>initial field</span>
          <select value={settings.initial} onChange={(e) => set({ initial: e.target.value as SimulationRequest["initial"] })}>
            <option value="uniform">uniform U_in, projected</option>
            <option value="rest">at rest</option>
          </select>
        </label>
      </div>
      <label className="check">
        <input type="checkbox" checked={settings.dt === null} onChange={(e) => set({ dt: e.target.checked ? null : 1e-3 })} />
        Adaptive Δt (advective + diffusive limit × CFL)
      </label>
      <PressureSolverField value={settings.pressure_solver} onChange={(pressure_solver) => set({ pressure_solver })} />
      {settings.dt !== null && (
        <>
          <NumberField label="Δt cap (s)" value={settings.dt} min={1e-12} onChange={(dt) => set({ dt })} />
          <p className="muted small">A Δt above the stability limit is rejected; it is never silently reduced.</p>
        </>
      )}
      {estimateError && <p className="warn small">Estimate unavailable: {estimateError}</p>}
      {est && (
        <div className={`stats small ${estimate.ok ? "" : "estimate-bad"}`}>
          <div>
            Estimate: ~<b>{est.steps.toLocaleString()}</b> steps (Δt ≈ {fmt(est.dt)} s) · {est.frames} frames · ~
            {est.iterations_per_step} PCG it/step
            {est.solver && (
              <>
                {" "}
                with <b>{SOLVER_LABELS[est.solver]}</b>
                {est.solver_requested === "auto" && " (Auto)"}
              </>
            )}
          </div>
          <div>
            Runtime ~<b>{fmtDuration(est.seconds)}</b> (budget {fmtDuration(estimate.budget.seconds)}) · output {fmtBytes(est.output_bytes)} ·
            memory ~{fmtBytes(est.memory_bytes)}
          </div>
          {est.alternative_solver && est.alternative_seconds !== undefined && (
            <div className="muted">
              {SOLVER_LABELS[est.alternative_solver]} would take ~{fmtDuration(est.alternative_seconds)}
              {est.multigrid_bytes ? ` · multigrid hierarchy ~${fmtBytes(est.multigrid_bytes)}` : ""}
            </div>
          )}
          {!estimate.ok && (
            <ul className="errors">
              {estimate.problems.map((p) => (
                <li key={p}>{p}</li>
              ))}
            </ul>
          )}
        </div>
      )}
    </div>
  );
}

/** Live progress of a queued or running simulation, with Cancel. */
export function SimulationProgressView({ job, onCancel }: { job: GridJob; onCancel: () => void }) {
  const p = job.progress;
  const fraction = p && p.duration > 0 ? Math.min(1, p.time / p.duration) : 0;
  return (
    <div className="stats small sim-progress" role="status">
      <div>
        {job.status === "queued" ? "Queued…" : "Simulating"} {p && <>t = <b>{fmt(p.time)}</b> / {fmt(p.duration)} s · step {p.step}</>}
      </div>
      <progress max={1} value={fraction} />
      {p?.iterations !== undefined && (
        <div className="muted">
          Δt {fmt(p.dt)} s · PCG {p.iterations} it (res {fmt(p.relative_residual)}) · max |div| {fmt(p.div_after_max)} 1/s · max |u|{" "}
          {fmt(p.max_speed)} m/s · {p.frames ?? 0} frames written
        </div>
      )}
      {job.estimate && <div className="muted">Estimated runtime ~{fmtDuration(job.estimate.seconds)}</div>}
      <button onClick={onCancel}>Cancel simulation</button>
    </div>
  );
}
