import type { FieldMetadata } from "../lib/fields/format";
import type { PathlineResult, TraceResult } from "../lib/streamlines/engine";
import type { Direction } from "../lib/streamlines/integrate";
import { WARN_CELLS_PER_FRAME, WITHHOLD_CELLS_PER_FRAME, type TemporalResolution } from "../lib/streamlines/pathlines";
import { MAX_DENSITY, MIN_DENSITY } from "../lib/streamlines/seeds";
import type { FlowMode, FlowSettings, FlowStatus, SeedMode } from "../lib/streamlines/useFlowVisualization";
import { NumberField, Panel } from "./Fields";

const fmt = (v: number) => (Math.abs(v) >= 1e4 || (Math.abs(v) < 1e-3 && v !== 0) ? v.toExponential(3) : String(Number(v.toPrecision(4))));

interface Props {
  settings: FlowSettings;
  onChange: (s: FlowSettings) => void;
  status: FlowStatus;
  streamlines: TraceResult | null; // only ever the displayed frame's
  lastStreamlineFrame: number | null;
  pathlines: PathlineResult | null;
  temporal: TemporalResolution | null;
  frame: number;
  frameTime: (frame: number) => number | null;
  velocityFrames: FieldMetadata[]; // sim_u frames (for the release-frame menu)
  colorNote: string;
  sliceAvailable: boolean;
  workerFrames: number[];
}

const stopsText = (stops: Record<string, number>) =>
  Object.entries(stops)
    .filter(([, n]) => n > 0)
    .map(([k, n]) => `${k} ${n}`)
    .join(", ") || "–";

/** Velocity visualization: instantaneous streamlines or time-integrated particle paths (Milestone 8). */
export function FlowControls(p: Props) {
  const { settings } = p;
  const set = (patch: Partial<FlowSettings>) => p.onChange({ ...settings, ...patch });
  const t = p.frameTime(p.frame);
  return (
    <Panel title="Velocity visualization">
      <label className="field">
        <span>Show</span>
        <select aria-label="Velocity visualization" value={settings.mode} onChange={(e) => set({ mode: e.target.value as FlowMode })}>
          <option value="off">Off</option>
          <option value="streamlines">Instantaneous streamlines</option>
          <option value="pathlines">Particle paths (time-integrated)</option>
        </select>
      </label>
      {settings.mode === "streamlines" && (
        <p className="muted small">
          <b>Instantaneous streamlines</b>: curves tangent to the velocity of the displayed frame only. In unsteady
          flow they are not the paths that particles follow.
        </p>
      )}
      {settings.mode === "pathlines" && (
        <p className="muted small">
          <b>Particle paths</b>: passive tracers released once at the release frame and carried by the time-dependent
          velocity, interpolated linearly in time between exported frames. Shown up to the displayed time, with
          markers at the tracers&apos; positions. Not streamlines, and not streaklines (no continuous release).
        </p>
      )}
      {settings.mode !== "off" && (
        <>
          <div className="vec3-inputs">
            <label className="field">
              <span>Seeds</span>
              <select aria-label="Seeds" value={settings.seedMode} onChange={(e) => set({ seedMode: e.target.value as SeedMode })}>
                <option value="slice" disabled={!p.sliceAvailable}>
                  Rake on the current slice
                </option>
                <option value="inlet">Inlet plane</option>
              </select>
            </label>
            {settings.mode === "streamlines" ? (
              <label className="field">
                <span>Direction</span>
                <select
                  aria-label="Integration direction"
                  value={settings.direction}
                  onChange={(e) => set({ direction: e.target.value as Direction })}
                >
                  <option value="both">Both ways</option>
                  <option value="forward">Downstream</option>
                  <option value="backward">Upstream</option>
                </select>
              </label>
            ) : (
              <label className="field">
                <span>Release at</span>
                <select
                  aria-label="Release frame"
                  value={settings.releaseFrame}
                  onChange={(e) => set({ releaseFrame: Number(e.target.value) })}
                >
                  {p.velocityFrames.map((f) => (
                    <option key={f.frame.index} value={f.frame.index}>
                      frame {f.frame.index + 1} (t = {fmt(f.frame.time_s ?? 0)} s)
                    </option>
                  ))}
                </select>
              </label>
            )}
          </div>
          <label className="field">
            <span>
              Seed density: {settings.density} × {settings.density}
            </span>
            <input
              type="range"
              aria-label="Seed density"
              min={MIN_DENSITY}
              max={MAX_DENSITY}
              value={settings.density}
              onChange={(e) => set({ density: Number(e.target.value) })}
            />
          </label>
          {settings.mode === "streamlines" ? (
            <NumberField label="Max length per direction (m)" value={settings.maxLength} min={1e-6} onChange={(maxLength) => set({ maxLength })} />
          ) : (
            <div className="row">
              <button onClick={() => set({ releaseFrame: p.frame })}>Release at the displayed frame</button>
              <label className="check inline">
                <input type="checkbox" checked={settings.showMarkers} onChange={(e) => set({ showMarkers: e.target.checked })} />
                Tracer markers
              </label>
            </div>
          )}
        </>
      )}

      {settings.mode === "pathlines" && p.temporal && (
        <div className={`stats small ${p.temporal.level === "ok" ? "" : "estimate-bad"}`}>
          <div>
            Tracers move up to <b>{fmt(p.temporal.cellsPerFrame)}</b> cells between exported frames (warn above{" "}
            {WARN_CELLS_PER_FRAME}, withheld above {WITHHOLD_CELLS_PER_FRAME}).
          </div>
          {p.temporal.level === "warn" && (
            <p className="warn">
              Coarse temporal resolution: flow changes between frames are not captured, so these paths are uncertain.
              Export frames every {fmt(p.temporal.intervalForWarn ?? 0)} s or less to remove this warning.
            </p>
          )}
          <div className="muted">
            Integration substeps (≤ ½ cell each) only control the error of interpolating between frames; they cannot
            recover flow behaviour that the exported frames did not capture. A small displacement per frame is
            necessary, not sufficient, for accurate paths.
          </div>
        </div>
      )}

      <div className="stats small" role="status">
        {(p.status.state === "unavailable" || p.status.state === "withheld") && <p className="warn">{p.status.message}</p>}
        {p.status.state === "error" && <p className="error">{p.status.message}</p>}
        {p.status.state === "computing" &&
          (settings.mode === "streamlines" ? (
            <div className="muted">
              Integrating streamlines for frame {p.status.frame + 1}…
              {p.lastStreamlineFrame !== null && p.lastStreamlineFrame !== p.frame && " (lines of other frames are hidden)"}
            </div>
          ) : (
            <div className="muted">Integrating particle paths from frame {p.status.frame + 1}…</div>
          ))}
        {settings.mode === "streamlines" && p.streamlines && (
          <>
            <div>
              <b>{(p.streamlines.offsets.length - 1).toLocaleString()}</b> streamlines from {p.streamlines.seeds.toLocaleString()}{" "}
              seeds · {p.streamlines.speeds.length.toLocaleString()} points · {fmt(p.streamlines.ms)} ms
              {p.streamlines.truncated && <span className="warn"> · point budget reached</span>}
            </div>
            <div>
              Velocity of frame <b>{p.streamlines.frame + 1}</b> (t = {fmt(p.frameTime(p.streamlines.frame) ?? 0)} s), the
              displayed frame
            </div>
            <div className="muted">Stopped by: {stopsText(p.streamlines.stops)}</div>
          </>
        )}
        {settings.mode === "streamlines" && !p.streamlines && p.status.state === "ready" && (
          <div className="muted">Streamlines for frame {p.frame + 1} are being prepared.</div>
        )}
        {settings.mode === "pathlines" && p.pathlines && (
          <>
            <div>
              <b>{(p.pathlines.offsets.length - 1).toLocaleString()}</b> tracers released at frame {p.pathlines.releaseFrame + 1} (t ={" "}
              {fmt(p.frameTime(p.pathlines.releaseFrame) ?? 0)} s), integrated to frame {p.pathlines.lastFrame + 1} ·{" "}
              {p.pathlines.speeds.length.toLocaleString()} points · {fmt(p.pathlines.ms)} ms
              {p.pathlines.truncated && <span className="warn"> · point budget reached: paths end early</span>}
            </div>
            <div>
              Shown up to t = <b>{fmt(t ?? 0)} s</b> (frame {p.frame + 1})
              {t !== null && t < (p.frameTime(p.pathlines.releaseFrame) ?? 0) && (
                <span className="warn"> · before the release time: nothing to show</span>
              )}
            </div>
            <div className="muted">Stopped by: {stopsText(p.pathlines.stops)} (end = reached the last frame)</div>
          </>
        )}
        {settings.mode !== "off" && (p.streamlines || p.pathlines) && <div className="muted">{p.colorNote}</div>}
        {settings.mode !== "off" && p.workerFrames.length > 0 && (
          <div className="muted">Worker holds velocity of {p.workerFrames.length} frame(s) (at most 3).</div>
        )}
      </div>
    </Panel>
  );
}
