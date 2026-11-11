import { useEffect, useMemo, useState } from "react";
import { MAX_LATTICE_SEGMENTS } from "./components/GridOverlay";
import { NumberField, Panel, Vec3Field } from "./components/Fields";
import { Viewer } from "./components/Viewer";
import { ApiError, createGridJob, getHealth, uploadModel, type GridMetadata, type ModelInfo } from "./lib/api";
import { decodeBase64, latticeSegmentCount, makeGridGeometry, type GridGeometry } from "./lib/grid";
import { loadModelFile, type LoadedModel } from "./lib/loaders";
import type { DisplayOptions, Domain, GizmoMode, ModelTransform, Resolution, TestBox, Vec3 } from "./lib/types";
import { MAX_CELLS_PER_AXIS, validateGridConfig } from "./lib/validation";

const DEFAULT_DOMAIN: Domain = { lx: 4, ly: 2, lz: 2 };
const DEFAULT_RESOLUTION: Resolution = { nx: 64, ny: 32, nz: 32 };
const DEFAULT_BOX: TestBox = { min: [1.0, 0.0, 0.75], max: [1.5, 0.5, 1.25] };

type BackendStatus = { state: "checking" } | { state: "ok"; engine: boolean; detail?: string | null } | { state: "down" };

interface GridResult {
  meta: GridMetadata;
  grid: GridGeometry;
  cells: Uint8Array;
  /** Serialized request, used to flag the result as stale when inputs change. */
  key: string;
}

export default function App() {
  const [backend, setBackend] = useState<BackendStatus>({ state: "checking" });
  const [domain, setDomain] = useState(DEFAULT_DOMAIN);
  const [resolution, setResolution] = useState(DEFAULT_RESOLUTION);
  const [boxEnabled, setBoxEnabled] = useState(true);
  const [testBox, setTestBox] = useState(DEFAULT_BOX);

  const [model, setModel] = useState<LoadedModel | null>(null);
  const [modelInfo, setModelInfo] = useState<ModelInfo | null>(null);
  const [modelMessage, setModelMessage] = useState<string | null>(null);
  const [transform, setTransform] = useState<ModelTransform>({ position: [1, 1, 1], rotation: [0, 0, 0], scale: [1, 1, 1] });
  const [gizmoMode, setGizmoMode] = useState<GizmoMode>("translate");
  const [modelOpacity, setModelOpacity] = useState(0.85);

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

  useEffect(() => {
    getHealth()
      .then((h) => setBackend({ state: "ok", engine: h.engine.available, detail: h.engine.detail }))
      .catch(() => setBackend({ state: "down" }));
  }, []);

  const activeBox = boxEnabled ? testBox : null;
  const errors = useMemo(() => validateGridConfig(domain, resolution, activeBox), [domain, resolution, activeBox]);
  const preview = useMemo(() => makeGridGeometry(domain, resolution), [domain, resolution]);
  const requestKey = JSON.stringify({ domain, resolution, activeBox });
  const stale = result !== null && result.key !== requestKey;

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
      setTransform({ position: [domain.lx / 2, domain.ly / 2, domain.lz / 2], rotation: [0, 0, 0], scale: [1, 1, 1] });
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

  function fitModel() {
    if (!model) return;
    const target = 0.3 * Math.min(domain.lx, domain.ly, domain.lz);
    const s = target / Math.max(model.size.x, model.size.y, model.size.z, 1e-12);
    setTransform((t) => ({ ...t, position: [domain.lx / 2, domain.ly / 2, domain.lz / 2], scale: [s, s, s] }));
  }

  async function generate() {
    setRunning(true);
    setJobError(null);
    const key = requestKey;
    try {
      const job = await createGridJob({
        domain,
        resolution,
        test_box: activeBox,
        model_id: modelInfo?.model_id ?? null,
        include_cells: true,
      });
      if (job.status !== "completed" || !job.result) {
        throw new ApiError(job.error ? `${job.error.code}: ${job.error.message}` : "grid job failed");
      }
      const meta = job.result;
      const cells = decodeBase64(meta.classification.data ?? "");
      if (cells.length !== meta.grid.cell_count) throw new ApiError("classification payload has wrong length");
      setResult({ meta, cells, key, grid: makeGridGeometry(meta.grid.domain, meta.grid.resolution) });
    } catch (e) {
      setJobError((e as Error).message);
    } finally {
      setRunning(false);
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
          {model && (
            <p className="muted">
              {model.name} — {model.triangles.toLocaleString()} triangles, size {fmtVec([model.size.x, model.size.y, model.size.z])}
              {modelInfo && <> · uploaded</>}
            </p>
          )}
          {modelMessage && <p className="warn">{modelMessage}</p>}
          <p className="muted small">Geometry is not voxelized in this milestone; the model is for placement only.</p>
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
              <button onClick={fitModel}>Fit to tunnel</button>
              <button onClick={() => setTransform((t) => ({ ...t, rotation: [0, 0, 0], scale: [1, 1, 1] }))}>Reset</button>
            </div>
            <label className="field">
              <span>Opacity</span>
              <input type="range" min={0.1} max={1} step={0.05} value={modelOpacity} onChange={(e) => setModelOpacity(Number(e.target.value))} />
            </label>
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
            {totalCells.toLocaleString()} cells · Δ = {fmtVec(preview.spacing)} m
          </p>
        </Panel>

        <Panel title="Test solid (box)">
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
        </Panel>

        <Panel title="Generate">
          {errors.length > 0 && (
            <ul className="errors">
              {errors.map((e) => (
                <li key={e}>{e}</li>
              ))}
            </ul>
          )}
          <button className="primary" disabled={errors.length > 0 || running} onClick={generate}>
            {running ? "Generating…" : "Generate grid"}
          </button>
          {jobError && <p className="error">{jobError}</p>}
          {result && (
            <div className="stats">
              {stale && <p className="warn">Inputs changed since this grid was generated.</p>}
              <div>
                Cells <b>{result.meta.grid.cell_count.toLocaleString()}</b>
              </div>
              <div>
                Fluid <b className="fluid">{result.meta.classification.fluid_count.toLocaleString()}</b> · Solid{" "}
                <b className="solid">{result.meta.classification.solid_count.toLocaleString()}</b>
              </div>
              {result.meta.warnings.map((w) => (
                <p key={w} className="warn">
                  {w}
                </p>
              ))}
            </div>
          )}
        </Panel>

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
        </Panel>
      </aside>

      <main className="viewport">
        <Viewer
          domain={domain}
          model={model}
          transform={transform}
          onTransformChange={setTransform}
          gizmoMode={gizmoMode}
          modelOpacity={modelOpacity}
          testBox={display.showTestBox ? activeBox : null}
          grid={{ preview, result: result ? { grid: result.grid, cells: result.cells } : null, display }}
        />
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

function fmtVec(v: Vec3 | number[]): string {
  return `(${v.map((x) => Number(x.toPrecision(4))).join(", ")})`;
}
