import { useEffect, useMemo, useRef, useState } from "react";
import { MAX_LATTICE_SEGMENTS } from "./components/GridOverlay";
import { FieldLegend } from "./components/FieldLegend";
import { NumberField, Panel, Vec3Field } from "./components/Fields";
import { FieldsNotice, FieldsPanel, type FieldViewSettings, type LastJobOutcome } from "./components/FieldsPanel";
import { PressureSolverField, SimulationProgressView, SimulationSettingsView } from "./components/SimulationControls";
import { Viewer } from "./components/Viewer";
import {
  ApiError,
  cancelGridJob,
  createGridJob,
  estimateGridJob,
  FINAL_JOB_STATUSES,
  getGridJob,
  SIMULATION_FIELD_IDS,
  getHealth,
  PROJECTION_FIELD_IDS,
  fetchField,
  getLimits,
  uploadModel,
  type EstimateResponse,
  type FieldId,
  type GridJob,
  type GridMetadata,
  type SimulationRequest,
  type ProjectionRequest,
  type MeshResult,
  type ModelInfo,
  type VoxelizationStats,
} from "./lib/api";
import { unionBounds } from "./lib/camera";
import {
  DEFAULT_CLEARANCES,
  currentSpacing,
  insideTunnel,
  proposeFit,
  validateClearanceFactors,
  type ClearanceFactors,
  type FitProposal,
  type GridLimits,
  type TunnelLayout,
} from "./lib/fit";
import {
  GEOMETRY_SOURCE_LABELS,
  MIN_CELLS_ACROSS,
  buildGridJobRequest,
  checkModelResolution,
  type GeometrySource,
} from "./lib/geometrySource";
import { defaultColormap, defaultRange } from "./lib/fields/colormap";
import { decodeField, type FieldMetadata, type ScalarFieldData } from "./lib/fields/format";
import { extractSlice } from "./lib/fields/sampling";
import { decodeBase64, latticeSegmentCount, makeGridGeometry, type GridGeometry } from "./lib/grid";
import { browserStorage, loadFieldPrefs, saveFieldPrefs } from "./lib/fieldPrefs";
import { loadModelFile, type LoadedModel } from "./lib/loaders";
import {
  UNIT_LABELS,
  boundsSize,
  modelMatrix,
  worldBounds,
  type Bounds,
  type LengthUnit,
} from "./lib/transform";
import type { DisplayOptions, Domain, GizmoMode, ModelTransform, Resolution, TestBox, Vec3 } from "./lib/types";
import { MAX_CELLS_PER_AXIS, validateGridConfig } from "./lib/validation";

const DEFAULT_DOMAIN: Domain = { lx: 4, ly: 2, lz: 2 };
const DEFAULT_RESOLUTION: Resolution = { nx: 64, ny: 32, nz: 32 };
const DEFAULT_BOX: TestBox = { min: [1.0, 0.0, 0.75], max: [1.5, 0.5, 1.25] };
/** Mirrors the backend defaults; replaced by /api/limits when the backend is reachable. */
const FALLBACK_LIMITS: GridLimits = { maxCells: 128 ** 3, maxCellsPerAxis: 512 };

/** Exportable fields; the last four come from one pressure projection of the initialized field. */
const FIELD_OPTIONS: [FieldId, string][] = [
  ["divergence_diagnostic", "Divergence diagnostic, initialized field (1/s)"],
  ["pressure", "Pressure after projection (Pa)"],
  ["divergence_before", "Divergence before projection (1/s)"],
  ["divergence_after", "Divergence after projection (1/s)"],
  ["velocity_magnitude", "Velocity magnitude after projection (m/s)"],
];

/** Time-dependent Navier-Stokes fields (Milestone 6): one AFFIELD file per field per frame. */
const SIMULATION_FIELD_OPTIONS: [FieldId, string][] = [
  ["sim_pressure", "Pressure p(t) (Pa)"],
  ["sim_velocity_magnitude", "Velocity magnitude |u|(t) (m/s)"],
  ["sim_divergence", "Divergence after projection (1/s)"],
  ["sim_u", "u(t), x-faces (m/s)"],
  ["sim_v", "v(t), y-faces (m/s)"],
  ["sim_w", "w(t), z-faces (m/s)"],
];

const POLL_INTERVAL_MS = 750;
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

const tunnelBounds = (d: Domain): Bounds => ({ min: [0, 0, 0], max: [d.lx, d.ly, d.lz] });

type BackendStatus = { state: "checking" } | { state: "ok"; engine: boolean; detail?: string | null } | { state: "down" };

interface GridResult {
  jobId: string;
  /** Numerical fields exported with this job (AFFIELD headers). */
  fields: FieldMetadata[];
  meta: GridMetadata;
  /** Model name when the mask came from the imported mesh. */
  modelName: string | null;
  grid: GridGeometry;
  cells: Uint8Array;
  /** Serialized request, used to flag the result as stale when inputs change. */
  key: string;
}

export default function App() {
  const [backend, setBackend] = useState<BackendStatus>({ state: "checking" });
  const [domain, setDomain] = useState(DEFAULT_DOMAIN);
  const [resolution, setResolution] = useState(DEFAULT_RESOLUTION);
  const [geometrySource, setGeometrySource] = useState<GeometrySource>("procedural_box");
  const [boxEnabled, setBoxEnabled] = useState(true);
  const [testBox, setTestBox] = useState(DEFAULT_BOX);

  const [limits, setLimits] = useState<{ value: GridLimits; fromServer: boolean }>({ value: FALLBACK_LIMITS, fromServer: false });
  const [model, setModel] = useState<LoadedModel | null>(null);
  const [units, setUnits] = useState<LengthUnit>("mm");
  const [modelInfo, setModelInfo] = useState<ModelInfo | null>(null);
  const [modelMessage, setModelMessage] = useState<string | null>(null);
  const [transform, setTransform] = useState<ModelTransform>({ position: [1, 1, 1], rotation: [0, 0, 0], scale: [1, 1, 1] });
  const [gizmoMode, setGizmoMode] = useState<GizmoMode>("translate");
  const [modelOpacity, setModelOpacity] = useState(0.85);
  const [clearances, setClearances] = useState<ClearanceFactors>(DEFAULT_CLEARANCES);
  const [proposal, setProposal] = useState<{ fit: FitProposal; key: string } | null>(null);
  const [fitMessage, setFitMessage] = useState<string | null>(null);
  const [frame, setFrame] = useState<{ id: number; bounds: Bounds }>({ id: 0, bounds: tunnelBounds(DEFAULT_DOMAIN) });

  const [display, setDisplay] = useState<DisplayOptions>({
    showLattice: true,
    showSolid: true,
    showFluid: false,
    showTestBox: true,
    slice: { enabled: false, axis: 2, index: 0 },
  });
  const [result, setResult] = useState<GridResult | null>(null);
  const [jobError, setJobError] = useState<string | null>(null);
  const [running, setRunning] = useState(false);
  // Field selections and projection settings persist across reloads (per-viewer convenience).
  const [initialPrefs] = useState(() => loadFieldPrefs(browserStorage()));
  const [requestedFields, setRequestedFields] = useState<FieldId[]>(initialPrefs.fields);
  const [projectionSettings, setProjectionSettings] = useState<ProjectionRequest>(initialPrefs.projection);
  const [simulationSettings, setSimulationSettings] = useState<SimulationRequest>(initialPrefs.simulation);
  useEffect(() => {
    saveFieldPrefs(browserStorage(), { fields: requestedFields, projection: projectionSettings, simulation: simulationSettings });
  }, [requestedFields, projectionSettings, simulationSettings]);
  // The job currently queued/running (simulations are asynchronous), for progress and Cancel.
  const [activeJob, setActiveJob] = useState<GridJob | null>(null);
  // Outcome of the most recent Generate, so the Fields panel can explain an empty result.
  const [lastJob, setLastJob] = useState<LastJobOutcome | null>(null);
  const toggleField = (id: FieldId, on: boolean) =>
    setRequestedFields((fs) => (on ? [...fs.filter((f) => f !== id), id] : fs.filter((f) => f !== id)));
  const projectionRequested = requestedFields.some((f) => PROJECTION_FIELD_IDS.includes(f));
  const simulationRequested = requestedFields.some((f) => SIMULATION_FIELD_IDS.includes(f));

  // Field visualization: selection, decoded data for the selected field, slice settings.
  const [selectedField, setSelectedField] = useState<string | null>(null);
  const [selectedFrame, setSelectedFrame] = useState(0);
  // Slice index is reset to the middle only when the field (not the frame) changes.
  const lastLoadedField = useRef<string | null>(null);
  const [fieldData, setFieldData] = useState<ScalarFieldData | null>(null);
  const [fieldStatus, setFieldStatus] = useState<{ loading: boolean; error: string | null }>({ loading: false, error: null });
  const [fieldView, setFieldView] = useState<FieldViewSettings>({
    visible: true,
    axis: 2,
    index: 0,
    rangeMode: "auto",
    manual: { min: -1, max: 1 },
    opacity: 0.9,
  });

  useEffect(() => {
    setFieldData(null);
    if (!result || !selectedField) return;
    let cancelled = false;
    setFieldStatus({ loading: true, error: null });
    fetchField(result.jobId, selectedField, selectedFrame)
      .then((buffer) => {
        if (cancelled) return;
        const decoded = decodeField(buffer);
        setFieldData(decoded);
        // Start at the middle slice of the current axis when a different field is shown.
        const key = `${result.jobId}:${selectedField}`;
        if (lastLoadedField.current !== key) {
          lastLoadedField.current = key;
          setFieldView((v) => ({ ...v, index: Math.floor(decoded.meta.shape[v.axis] / 2) }));
        }
        setFieldStatus({ loading: false, error: null });
      })
      .catch((e) => !cancelled && setFieldStatus({ loading: false, error: (e as Error).message }));
    return () => {
      cancelled = true;
    };
  }, [result, selectedField, selectedFrame]);

  const fieldColormap = fieldData ? defaultColormap(fieldData.meta) : null;
  const fieldRange =
    fieldData && fieldColormap
      ? fieldView.rangeMode === "auto"
        ? defaultRange(fieldData.meta, fieldColormap)
        : fieldView.manual
      : null;
  const fieldIndex = fieldData ? Math.min(fieldView.index, fieldData.meta.shape[fieldView.axis] - 1) : 0;
  const fieldPlane = useMemo(
    () => (fieldData ? extractSlice(fieldData, fieldView.axis, fieldIndex).planeCoordinate : null),
    [fieldData, fieldView.axis, fieldIndex],
  );

  useEffect(() => {
    getHealth()
      .then((h) => setBackend({ state: "ok", engine: h.engine.available, detail: h.engine.detail }))
      .catch(() => setBackend({ state: "down" }));
    getLimits()
      .then((l) => setLimits({ value: { maxCells: l.max_cells, maxCellsPerAxis: l.max_cells_per_axis }, fromServer: true }))
      .catch(() => undefined);
  }, []);

  // The procedural box is only ever sent in procedural mode; never combined with the mesh.
  const activeBox = geometrySource === "procedural_box" && boxEnabled ? testBox : null;
  const modelBounds = useMemo(
    () => (model ? worldBounds(model.geometry.getAttribute("position").array, modelMatrix(units, model.pivot, transform)) : null),
    [model, units, transform],
  );
  const transformErrors = transform.scale.some((s) => !(s > 0)) ? ["model scale must be > 0 on every axis"] : [];
  const sourceErrors =
    geometrySource === "mesh" && !modelInfo ? ["imported mesh mode needs a model that has been uploaded to the backend"] : [];
  const errors = [
    ...validateGridConfig(domain, resolution, activeBox, limits.value.maxCells),
    ...transformErrors,
    ...sourceErrors,
  ];
  const resolutionCheck =
    geometrySource === "mesh" && modelBounds ? checkModelResolution(modelBounds, currentSpacing(domain, resolution)) : null;
  const preview = useMemo(() => makeGridGeometry(domain, resolution), [domain, resolution]);
  const requestKey = JSON.stringify({
    domain,
    resolution,
    geometrySource,
    activeBox,
    mesh: geometrySource === "mesh" ? { id: modelInfo?.model_id, units, transform } : null,
  });
  const stale = result !== null && result.key !== requestKey;
  // Preflight estimate for simulation runs (debounced; nothing is run).
  const jobRequest = useMemo(() => {
    const mesh = modelInfo && model ? { modelId: modelInfo.model_id, units, pivot: model.pivot, transform } : null;
    return buildGridJobRequest(geometrySource, domain, resolution, activeBox, mesh, requestedFields, projectionSettings, simulationSettings);
  }, [geometrySource, domain, resolution, activeBox, modelInfo, model, units, transform, requestedFields, projectionSettings, simulationSettings]);
  const [estimate, setEstimate] = useState<{ value: EstimateResponse | null; error: string | null }>({ value: null, error: null });
  const estimateKey = simulationRequested && errors.length === 0 ? JSON.stringify(jobRequest) : null;
  useEffect(() => {
    if (!estimateKey) {
      setEstimate({ value: null, error: null });
      return;
    }
    let cancelled = false;
    const timer = setTimeout(() => {
      estimateGridJob(JSON.parse(estimateKey))
        .then((value) => !cancelled && setEstimate({ value, error: null }))
        .catch((e) => !cancelled && setEstimate({ value: null, error: (e as Error).message }));
    }, 400);
    return () => {
      cancelled = true;
      clearTimeout(timer);
    };
  }, [estimateKey]);

  const fitKey = JSON.stringify({ domain, resolution, units, transform, clearances, limits, model: model?.name });
  const activeProposal = proposal && proposal.key === fitKey ? proposal.fit : null;

  const sliceGrid = result?.grid ?? preview;
  const sliceMax = [sliceGrid.resolution.nx, sliceGrid.resolution.ny, sliceGrid.resolution.nz][display.slice.axis] - 1;
  useEffect(() => {
    if (display.slice.index > sliceMax) setDisplay((d) => ({ ...d, slice: { ...d.slice, index: sliceMax } }));
  }, [sliceMax, display.slice.index]);

  async function onFile(file: File) {
    setModelMessage(null);
    setModelInfo(null);
    try {
      const loaded = await loadModelFile(file);
      setModel(loaded);
      // The pivot (raw bbox center) is placed at the tunnel center; units come from the selector.
      setTransform({ position: [domain.lx / 2, domain.ly / 2, domain.lz / 2], rotation: [0, 0, 0], scale: [1, 1, 1] });
      setProposal(null);
      setFitMessage(null);
    } catch (e) {
      setModelMessage(`Could not read model: ${(e as Error).message}`);
      return;
    }
    try {
      setModelInfo(await uploadModel(file));
    } catch (e) {
      setModelMessage(`Loaded locally; upload failed: ${(e as Error).message}`);
    }
  }

  function fitTunnel() {
    if (!modelBounds) return;
    setFitMessage(null);
    try {
      const fit = proposeFit(modelBounds, transform.position, clearances, currentSpacing(domain, resolution), limits.value);
      setProposal({ fit, key: fitKey });
    } catch (e) {
      setProposal(null);
      setFitMessage(`Cannot fit tunnel: ${(e as Error).message}`);
    }
  }

  /** Applies a fitted layout: tunnel size, grid counts and model position only (never scale/rotation/units). */
  function applyLayout(layout: TunnelLayout) {
    setDomain(layout.domain);
    setResolution(layout.resolution);
    setTransform((t) => ({ ...t, position: layout.position }));
    setProposal(null);
    setFrame((f) => ({ id: f.id + 1, bounds: unionBounds(tunnelBounds(layout.domain), layout.modelBounds) }));
    const box = boxEnabled ? testBox : null;
    if (box && validateGridConfig(layout.domain, layout.resolution, box).length > 0) {
      setBoxEnabled(false);
      setFitMessage("Test box disabled: it no longer lies inside the resized tunnel.");
    }
  }

  function reframe() {
    const bounds = modelBounds ? unionBounds(tunnelBounds(domain), modelBounds) : tunnelBounds(domain);
    setFrame((f) => ({ id: f.id + 1, bounds }));
  }

  async function generate() {
    setRunning(true);
    setJobError(null);
    const key = requestKey;
    const requested = [...requestedFields];
    try {
      let job = await createGridJob(jobRequest);
      // Simulations run asynchronously: poll until the job reaches a final state.
      while (!FINAL_JOB_STATUSES.includes(job.status)) {
        setActiveJob(job);
        await sleep(POLL_INTERVAL_MS);
        job = await getGridJob(job.job_id);
      }
      setActiveJob(null);
      if (job.status === "cancelled") {
        setLastJob({ status: "cancelled", requested, error: "the simulation was cancelled; its frames were discarded" });
        return;
      }
      if ((job.status !== "completed" && job.status !== "stopped") || !job.result) {
        throw new ApiError(job.error ? `${job.error.code}: ${job.error.message}` : "grid job failed");
      }
      setLastJob({
        status: job.status,
        requested,
        error: job.status === "stopped" && job.error ? job.error.message : null,
      });
      const meta = job.result;
      const cells = decodeBase64(meta.classification.data ?? "");
      if (cells.length !== meta.grid.cell_count) throw new ApiError("classification payload has wrong length");
      setResult({
        jobId: job.job_id,
        fields: job.fields,
        meta,
        cells,
        key,
        grid: makeGridGeometry(meta.grid.domain, meta.grid.resolution),
        modelName: geometrySource === "mesh" ? (model?.name ?? null) : null,
      });
      setSelectedField(job.fields[0]?.name ?? null);
      setSelectedFrame(0);
    } catch (e) {
      setJobError((e as Error).message);
      setLastJob({ status: "failed", requested, error: (e as Error).message });
    } finally {
      setActiveJob(null);
      setRunning(false);
    }
  }

  async function cancelActiveJob() {
    if (!activeJob) return;
    try {
      await cancelGridJob(activeJob.job_id);
    } catch (e) {
      // 409: it finished in the meantime; the poll loop picks up the final state.
      if (!(e instanceof ApiError && /already/.test(e.message))) setJobError((e as Error).message);
    }
  }

  const totalCells = resolution.nx * resolution.ny * resolution.nz;
  const latticeTooBig = latticeSegmentCount(sliceGrid.resolution) > MAX_LATTICE_SEGMENTS;

  return (
    <div className="app">
      <aside className="sidebar">
        <header>
          <h1>AeroFlow</h1>
          <BackendBadge status={backend} />
        </header>

        <Panel title="Model">
          <input
            type="file"
            accept=".stl,.obj"
            onChange={(e) => {
              const f = e.target.files?.[0];
              if (f) void onFile(f);
              e.target.value = "";
            }}
          />
          <label className="field units">
            <span>Model units</span>
            <select value={units} onChange={(e) => setUnits(e.target.value as LengthUnit)}>
              {(Object.keys(UNIT_LABELS) as LengthUnit[]).map((u) => (
                <option key={u} value={u}>
                  {UNIT_LABELS[u]} ({u})
                </option>
              ))}
            </select>
          </label>
          {model && (
            <p className="muted">
              {model.name} — {model.triangles.toLocaleString()} triangles
              {modelInfo && <> · uploaded</>}
              <br />
              File extent: {fmtVec([model.rawSize.x, model.rawSize.y, model.rawSize.z])} {units}
            </p>
          )}
          {model && modelBounds && (
            <div className="stats">
              <div>
                Physical size: <b>{fmtVec(boundsSize(modelBounds))} m</b>
              </div>
              <div className="muted small">
                World bbox {fmtVec(modelBounds.min)} → {fmtVec(modelBounds.max)} m
              </div>
              {!insideTunnel(modelBounds, domain) && <p className="warn">Model touches or extends past the tunnel walls.</p>}
            </div>
          )}
          {modelMessage && <p className="warn">{modelMessage}</p>}
          <p className="muted small">To build the solid-cell mask from this model, choose <b>Imported mesh</b> under Solid geometry source.</p>
        </Panel>

        {model && (
          <Panel title="Transform">
            <div className="segmented">
              {(["translate", "rotate", "scale", "none"] as const).map((m) => (
                <button key={m} className={gizmoMode === m ? "active" : ""} onClick={() => setGizmoMode(m)}>
                  {m}
                </button>
              ))}
            </div>
            <Vec3Field label="Position (m)" value={transform.position} step={0.05} onChange={(position) => setTransform((t) => ({ ...t, position }))} />
            <Vec3Field label="Rotation (deg)" value={transform.rotation} step={5} onChange={(rotation) => setTransform((t) => ({ ...t, rotation }))} />
            <Vec3Field label="Scale" value={transform.scale} step={0.1} min={1e-9} onChange={(scale) => setTransform((t) => ({ ...t, scale }))} />
            <div className="row">
              <button onClick={() => setTransform((t) => ({ ...t, rotation: [0, 0, 0], scale: [1, 1, 1] }))}>Reset rotation &amp; scale</button>
            </div>
            <label className="field">
              <span>Opacity</span>
              <input type="range" min={0.1} max={1} step={0.05} value={modelOpacity} onChange={(e) => setModelOpacity(Number(e.target.value))} />
            </label>
          </Panel>
        )}

        {model && (
          <Panel title="Fit tunnel to model">
            <p className="muted small">
              Clearances are multiples of D = largest model dimension ({modelBounds ? fmtNum(Math.max(...boundsSize(modelBounds))) : "–"} m). The
              model keeps its size, scale and orientation; only the tunnel and model position change.
            </p>
            <div className="vec3-inputs">
              {(["upstream", "downstream", "lateral"] as const).map((k) => (
                <NumberField key={k} label={k} value={clearances[k]} min={1e-6} step={0.5} onChange={(v) => setClearances((c) => ({ ...c, [k]: v }))} />
              ))}
            </div>
            <div className="vec3-inputs">
              {(["top", "bottom"] as const).map((k) => (
                <NumberField key={k} label={k} value={clearances[k]} min={1e-6} step={0.5} onChange={(v) => setClearances((c) => ({ ...c, [k]: v }))} />
              ))}
            </div>
            <button
              className="primary"
              style={{ marginTop: 8 }}
              disabled={!modelBounds || validateClearanceFactors(clearances).length > 0 || transformErrors.length > 0}
              onClick={fitTunnel}
            >
              Fit Tunnel to Model
            </button>
            {fitMessage && <p className="warn">{fitMessage}</p>}
            {activeProposal && (
              <FitProposalView proposal={activeProposal} limits={limits} onApply={applyLayout} onCancel={() => setProposal(null)} />
            )}
          </Panel>
        )}

        <Panel title="Wind tunnel (m)">
          <p className="muted small">Domain [0, L] on each axis. Flow along +x, y up.</p>
          <Vec3Field
            label="Size"
            labels={["Lx", "Ly", "Lz"]}
            value={[domain.lx, domain.ly, domain.lz]}
            min={1e-9}
            step={0.1}
            onChange={([lx, ly, lz]) => setDomain({ lx, ly, lz })}
          />
        </Panel>

        <Panel title="Grid resolution">
          <div className="vec3-inputs">
            {(["nx", "ny", "nz"] as const).map((k) => (
              <NumberField
                key={k}
                label={k}
                value={resolution[k]}
                integer
                min={1}
                max={MAX_CELLS_PER_AXIS}
                step={1}
                onChange={(v) => setResolution((r) => ({ ...r, [k]: v }))}
              />
            ))}
          </div>
          <p className="muted small">
            {totalCells.toLocaleString()} cells · Δ = {fmtVec(preview.spacing)} m · limit {limits.value.maxCells.toLocaleString()}
            {!limits.fromServer && " (assumed)"}
          </p>
        </Panel>

        <Panel title="Solid geometry source">
          <div className="segmented">
            {(["procedural_box", "mesh"] as const).map((src) => (
              <button
                key={src}
                className={geometrySource === src ? "active" : ""}
                disabled={src === "mesh" && !modelInfo}
                title={src === "mesh" && !modelInfo ? "Import a model (uploaded to the backend) first" : undefined}
                onClick={() => setGeometrySource(src)}
              >
                {GEOMETRY_SOURCE_LABELS[src]}
              </button>
            ))}
          </div>
          {geometrySource === "procedural_box" ? (
            <>
              <label className="check">
                <input type="checkbox" checked={boxEnabled} onChange={(e) => setBoxEnabled(e.target.checked)} />
                Procedural solid box
              </label>
              {boxEnabled && (
                <>
                  <Vec3Field label="Min (m)" value={testBox.min} step={0.05} onChange={(min) => setTestBox((b) => ({ ...b, min }))} />
                  <Vec3Field label="Max (m)" value={testBox.max} step={0.05} onChange={(max) => setTestBox((b) => ({ ...b, max }))} />
                  <p className="muted small">A cell is solid when its center lies in [min, max).</p>
                </>
              )}
            </>
          ) : (
            <>
              <p className="muted small">
                Voxelizing <b>{model?.name}</b> in {units} with its current transform. A cell is solid when its center is
                inside the closed mesh (centers on the surface count as solid). The procedural box is not used.
              </p>
              {resolutionCheck?.poorlyResolved && (
                <p className="warn">
                  Poorly resolved: the model&apos;s thinnest dimension spans only {fmtNum(resolutionCheck.minCellsAcross)} cells
                  (fewer than {MIN_CELLS_ACROSS}). Consider a finer grid; it will not be changed automatically.
                </p>
              )}
            </>
          )}
        </Panel>

        <Panel title="Generate">
          {errors.length > 0 && (
            <ul className="errors">
              {errors.map((e) => (
                <li key={e}>{e}</li>
              ))}
            </ul>
          )}
          <div className="field-requests">
            <div className="muted small">Export fields (opt-in)</div>
            {FIELD_OPTIONS.map(([id, label]) => (
              <label key={id} className="check">
                <input type="checkbox" checked={requestedFields.includes(id)} onChange={(e) => toggleField(id, e.target.checked)} />
                {label}
              </label>
            ))}
            {projectionRequested && (
              <div className="projection-settings">
                <div className="vec3-inputs">
                  <NumberField
                    label="U_in (m/s)"
                    value={projectionSettings.inlet_velocity}
                    min={0}
                    onChange={(inlet_velocity) => setProjectionSettings((p) => ({ ...p, inlet_velocity }))}
                  />
                  <NumberField
                    label="ρ (kg/m³)"
                    value={projectionSettings.density}
                    min={1e-9}
                    onChange={(density) => setProjectionSettings((p) => ({ ...p, density }))}
                  />
                </div>
                <div className="vec3-inputs">
                  <NumberField
                    label="tolerance"
                    value={projectionSettings.tolerance}
                    min={1e-15}
                    max={0.5}
                    onChange={(tolerance) => setProjectionSettings((p) => ({ ...p, tolerance }))}
                  />
                  <NumberField
                    label="max iterations"
                    value={projectionSettings.max_iterations}
                    integer
                    min={1}
                    max={1000000}
                    onChange={(max_iterations) => setProjectionSettings((p) => ({ ...p, max_iterations }))}
                  />
                </div>
                <label className="check">
                  <input
                    type="checkbox"
                    checked={projectionSettings.dt === null}
                    onChange={(e) => setProjectionSettings((p) => ({ ...p, dt: e.target.checked ? null : 1e-3 }))}
                  />
                  Δt from CFL 0.5 (pressure scale p = ρφ/Δt)
                </label>
                {projectionSettings.dt !== null && (
                  <NumberField
                    label="Δt (s)"
                    value={projectionSettings.dt}
                    min={1e-12}
                    onChange={(dt) => setProjectionSettings((p) => ({ ...p, dt }))}
                  />
                )}
                <PressureSolverField
                  value={projectionSettings.pressure_solver}
                  onChange={(pressure_solver) => setProjectionSettings((p) => ({ ...p, pressure_solver }))}
                />
              </div>
            )}
            <div className="muted small">Time-dependent Navier–Stokes (runs in the background)</div>
            {SIMULATION_FIELD_OPTIONS.map(([id, label]) => (
              <label key={id} className="check">
                <input type="checkbox" checked={requestedFields.includes(id)} onChange={(e) => toggleField(id, e.target.checked)} />
                {label}
              </label>
            ))}
            {simulationRequested && (
              <SimulationSettingsView
                settings={simulationSettings}
                onChange={setSimulationSettings}
                estimate={estimate.value}
                estimateError={estimate.error}
              />
            )}
          </div>
          <button
            className="primary"
            disabled={errors.length > 0 || running || (simulationRequested && estimate.value?.ok === false)}
            onClick={generate}
          >
            {running ? (activeJob ? "Simulating…" : "Generating…") : simulationRequested ? "Generate & simulate" : "Generate grid"}
          </button>
          {activeJob && <SimulationProgressView job={activeJob} onCancel={cancelActiveJob} />}
          {jobError && <p className="error">{jobError}</p>}
          {result && (
            <div className="stats">
              {stale && <p className="warn">Inputs changed since this grid was generated.</p>}
              <div>
                Cells <b>{result.meta.grid.cell_count.toLocaleString()}</b>
              </div>
              <div>
                Source{" "}
                <b>
                  {result.meta.geometry_source === "mesh"
                    ? `Imported mesh${result.modelName ? ` (${result.modelName})` : ""}`
                    : result.meta.geometry_source === "procedural_box"
                      ? "Procedural box"
                      : "None (all fluid)"}
                </b>
              </div>
              <div>
                Fluid <b className="fluid">{result.meta.classification.fluid_count.toLocaleString()}</b> · Solid{" "}
                <b className="solid">{result.meta.classification.solid_count.toLocaleString()}</b>
              </div>
              <div className="muted small">
                Δ = {fmtVec([result.meta.grid.spacing.dx, result.meta.grid.spacing.dy, result.meta.grid.spacing.dz])} m
              </div>
              {result.meta.mesh && result.meta.voxelization && (
                <MeshResultSummary mesh={result.meta.mesh} stats={result.meta.voxelization} />
              )}
              {result.meta.warnings.map((w) => (
                <p key={w} className="warn">
                  {w}
                </p>
              ))}
            </div>
          )}
        </Panel>

        {lastJob &&
          (lastJob.status === "failed" ||
            lastJob.status === "cancelled" ||
            lastJob.status === "stopped" ||
            (result !== null && result.fields.length === 0)) && (
            <FieldsNotice
              outcome={lastJob}
              showingPreviousFields={(lastJob.status === "failed" || lastJob.status === "cancelled") && (result?.fields.length ?? 0) > 0}
            />
          )}
        {result && result.fields.length > 0 && (
          <FieldsPanel
            fields={result.fields}
            selected={selectedField}
            onSelect={(name) => {
              setSelectedField(name);
              setSelectedFrame(0);
            }}
            frame={selectedFrame}
            onFrameChange={setSelectedFrame}
            data={fieldData}
            loading={fieldStatus.loading}
            error={fieldStatus.error}
            view={fieldView}
            onViewChange={setFieldView}
            colormap={fieldColormap}
            range={fieldRange}
            planeCoordinate={fieldPlane}
            projection={result.meta.projection ?? null}
            simulation={result.meta.simulation ?? null}
          />
        )}

        <Panel title="Display">
          <label className="check">
            <input type="checkbox" checked={display.showLattice} onChange={(e) => setDisplay({ ...display, showLattice: e.target.checked })} />
            Grid lattice
          </label>
          {display.showLattice && latticeTooBig && !display.slice.enabled && (
            <p className="muted small">Lattice too dense to draw in 3D; enable a slice to view it.</p>
          )}
          <label className="check">
            <input type="checkbox" checked={display.showSolid} onChange={(e) => setDisplay({ ...display, showSolid: e.target.checked })} />
            Solid cells
          </label>
          <label className="check">
            <input type="checkbox" checked={display.showFluid} onChange={(e) => setDisplay({ ...display, showFluid: e.target.checked })} />
            Fluid cell centers
          </label>
          <label className="check">
            <input type="checkbox" checked={display.showTestBox} onChange={(e) => setDisplay({ ...display, showTestBox: e.target.checked })} />
            Test box outline
          </label>
          <label className="check">
            <input
              type="checkbox"
              checked={display.slice.enabled}
              onChange={(e) => setDisplay({ ...display, slice: { ...display.slice, enabled: e.target.checked } })}
            />
            Slice
          </label>
          {display.slice.enabled && (
            <div className="row">
              <select
                value={display.slice.axis}
                onChange={(e) => setDisplay({ ...display, slice: { ...display.slice, axis: Number(e.target.value) as 0 | 1 | 2 } })}
              >
                <option value={0}>i (x)</option>
                <option value={1}>j (y)</option>
                <option value={2}>k (z)</option>
              </select>
              <input
                type="range"
                min={0}
                max={sliceMax}
                value={Math.min(display.slice.index, sliceMax)}
                onChange={(e) => setDisplay({ ...display, slice: { ...display.slice, index: Number(e.target.value) } })}
              />
              <span className="mono">{Math.min(display.slice.index, sliceMax)}</span>
            </div>
          )}
          <button onClick={reframe}>Reframe view</button>
        </Panel>
      </aside>

      <main className="viewport">
        <Viewer
          domain={domain}
          model={model}
          units={units}
          transform={transform}
          onTransformChange={setTransform}
          gizmoMode={gizmoMode}
          modelOpacity={modelOpacity}
          testBox={display.showTestBox ? activeBox : null}
          grid={{ preview, result: result ? { grid: result.grid, cells: result.cells } : null, display }}
          frame={frame}
          fieldSlice={
            fieldData && fieldRange && fieldColormap && fieldView.visible
              ? {
                  field: fieldData,
                  axis: fieldView.axis,
                  index: fieldIndex,
                  range: fieldRange,
                  colormap: fieldColormap,
                  opacity: fieldView.opacity,
                }
              : null
          }
        />
        {fieldData && fieldRange && fieldColormap && fieldView.visible && fieldPlane !== null && (
          <FieldLegend
            meta={fieldData.meta}
            range={fieldRange}
            colormap={fieldColormap}
            axis={fieldView.axis}
            index={fieldIndex}
            planeCoordinate={fieldPlane}
          />
        )}
      </main>
    </div>
  );
}

function BackendBadge({ status }: { status: BackendStatus }) {
  if (status.state === "checking") return <span className="badge">checking…</span>;
  if (status.state === "down") return <span className="badge bad" title="Start the FastAPI backend on port 8000">backend offline</span>;
  if (!status.engine) return <span className="badge warn" title={status.detail ?? ""}>engine missing</span>;
  return <span className="badge ok">backend ok</span>;
}

function fmtNum(x: number): string {
  return String(Number(x.toPrecision(4)));
}

function fmtVec(v: Vec3 | number[]): string {
  return `(${v.map(fmtNum).join(", ")})`;
}

function MeshResultSummary({ mesh, stats }: { mesh: MeshResult; stats: VoxelizationStats }) {
  const err = mesh.volume_relative_error;
  return (
    <div className="small">
      <div>
        Voxelization <b>{fmtNum(stats.bvh_build_ms + stats.classify_ms)} ms</b> ({stats.triangles.toLocaleString()} triangles,{" "}
        {stats.ray_triangle_tests.toLocaleString()} ray–triangle tests)
      </div>
      <div>
        Volume: mesh {mesh.volume !== null ? `${fmtNum(mesh.volume)} m³` : "n/a"} · voxels {fmtNum(mesh.voxel_volume)} m³
        {err !== null && (
          <>
            {" "}
            · error <b>{fmtNum(100 * err)}%</b>
          </>
        )}
      </div>
      {(stats.fallback_cells > 0 || stats.surface_cells > 0) && (
        <div className="muted">
          {stats.fallback_cells.toLocaleString()} cells re-cast after ambiguous crossings · {stats.surface_cells.toLocaleString()} on the
          surface
        </div>
      )}
    </div>
  );
}

function LayoutSummary({ layout }: { layout: TunnelLayout }) {
  const { nx, ny, nz } = layout.resolution;
  const g = layout.gaps;
  return (
    <div className="small">
      <div>
        Tunnel <b>{fmtVec([layout.domain.lx, layout.domain.ly, layout.domain.lz])} m</b>
      </div>
      <div>
        Grid <b>
          {nx} × {ny} × {nz}
        </b>{" "}
        = <b>{layout.totalCells.toLocaleString()}</b> cells · Δ = {fmtVec(layout.spacing)} m
      </div>
      <div className="muted">
        Gaps: up {fmtNum(g.upstream)} · down {fmtNum(g.downstream)} · bottom {fmtNum(g.bottom)} · top {fmtNum(g.top)} · z {fmtNum(g.zMin)}/
        {fmtNum(g.zMax)} m
      </div>
    </div>
  );
}

function FitProposalView({
  proposal,
  limits,
  onApply,
  onCancel,
}: {
  proposal: FitProposal;
  limits: { value: GridLimits; fromServer: boolean };
  onApply: (layout: TunnelLayout) => void;
  onCancel: () => void;
}) {
  const { required, coarser } = proposal;
  return (
    <div className="proposal">
      <div className="proposal-title">At current spacing</div>
      <LayoutSummary layout={required} />
      {proposal.withinLimits ? (
        <div className="row">
          <button className="primary" onClick={() => onApply(required)}>
            Apply
          </button>
          <button onClick={onCancel}>Cancel</button>
        </div>
      ) : (
        <>
          <p className="warn">
            Exceeds the grid budget ({limits.value.maxCells.toLocaleString()} cells, {limits.value.maxCellsPerAxis} per axis
            {!limits.fromServer && ", assumed"}).
          </p>
          {coarser && (
            <>
              <div className="proposal-title">Coarser grid option</div>
              <LayoutSummary layout={coarser} />
            </>
          )}
          <div className="row wrap">
            {coarser && (
              <button className="primary" onClick={() => onApply(coarser)}>
                Use coarser grid
              </button>
            )}
            <button onClick={() => onApply(required)} title="Apply the tunnel at the current spacing; then reduce the tunnel or grid yourself">
              Adjust manually
            </button>
            <button onClick={onCancel}>Cancel</button>
          </div>
        </>
      )}
    </div>
  );
}
