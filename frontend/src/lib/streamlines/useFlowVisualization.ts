import { useEffect, useMemo, useRef, useState } from "react";
import type { FieldMetadata } from "../fields/format";
import type { PathlineResult, TraceResult } from "./engine";
import { gridGeometry, pathlineFrames, streamlinesForFrame, VELOCITY_COMPONENTS } from "./flowDisplay";
import { DEFAULT_TRACE, type Direction, type TraceOptions } from "./integrate";
import { DEFAULT_PATH_OPTIONS, temporalResolution, type TemporalResolution } from "./pathlines";
import { inletSeeds, sliceRakeSeeds } from "./seeds";
import type { WorkerRequest } from "./streamlineWorker";

export type FlowMode = "off" | "streamlines" | "pathlines";
export type SeedMode = "slice" | "inlet";

export interface FlowSettings {
  mode: FlowMode;
  seedMode: SeedMode;
  density: number; // seeds per in-plane axis
  // Instantaneous streamlines
  maxLength: number; // m per direction
  direction: Direction;
  // Particle paths
  releaseFrame: number;
  showMarkers: boolean;
}

export type FlowStatus =
  | { state: "off" }
  | { state: "unavailable"; message: string }
  | { state: "withheld"; message: string }
  | { state: "computing"; frame: number }
  | { state: "ready" }
  | { state: "error"; message: string };

interface Args {
  jobId: string | null;
  fields: FieldMetadata[];
  frame: number; // displayed frame index
  solid: Uint8Array | null;
  settings: FlowSettings;
  slice: { axis: 0 | 1 | 2; coordinate: number } | null;
}

/**
 * Instantaneous streamlines or particle paths, computed in a Web Worker that loads the velocity
 * frames itself (no velocity arrays are copied from, or retained by, the main thread).
 *
 * Streamlines: one request per displayed frame; while one runs, newer frames replace a single
 * pending request (settings changes cancel the running one). Only the result for the displayed
 * frame is returned, so lines from another frame are never shown as the current frame's.
 * Particle paths: computed once per release frame / seeds, from the release frame to the last frame;
 * the display reveals them up to the displayed frame's time. Withheld when the exported frames are
 * too far apart in time (temporalResolution).
 */
export function useFlowVisualization({ jobId, fields, frame, solid, settings, slice }: Args) {
  const [stream, setStream] = useState<TraceResult | null>(null);
  const [paths, setPaths] = useState<PathlineResult | null>(null);
  const [status, setStatus] = useState<FlowStatus>({ state: "off" });
  const [workerFrames, setWorkerFrames] = useState<number[]>([]);
  const [workerEpoch, setWorkerEpoch] = useState(0); // bumps when a worker is created
  const worker = useRef<Worker | null>(null);
  const busy = useRef(false);
  const pending = useRef<(() => void) | null>(null);
  const runningKey = useRef<string | null>(null);
  const nextJob = useRef(1);
  const latest = useRef(0);
  const active = settings.mode !== "off";

  const componentMeta = useMemo(() => fields.find((f) => f.name === "sim_u") ?? null, [fields]);
  const geometry = useMemo(() => (componentMeta && solid ? gridGeometry(componentMeta, solid) : null), [componentMeta, solid]);
  const frameAvailable = useMemo(
    () => VELOCITY_COMPONENTS.every((c) => fields.some((f) => f.name === c && f.frame.index === frame)),
    [fields, frame],
  );
  const releaseFrames = useMemo(() => pathlineFrames(fields, settings.releaseFrame), [fields, settings.releaseFrame]);
  const temporal: TemporalResolution | null = useMemo(
    () => (geometry && releaseFrames.length >= 2 ? temporalResolution(releaseFrames, geometry.h) : null),
    [geometry, releaseFrames],
  );

  // Worker lifecycle: one worker while a mode is active; it is told which job's files to read.
  useEffect(() => {
    if (!active || !jobId || !solid) return;
    const w = new Worker(new URL("./streamlineWorker.ts", import.meta.url), { type: "module" });
    worker.current = w;
    w.postMessage({ type: "dataset", key: jobId, apiJobId: jobId, solid } satisfies WorkerRequest);
    setWorkerEpoch((e) => e + 1);
    w.onmessage = (e: MessageEvent) => {
      busy.current = false;
      const msg = e.data as { type: string; result?: TraceResult | PathlineResult; jobId: number; message?: string; cachedFrames?: number[] };
      if (msg.cachedFrames) setWorkerFrames(msg.cachedFrames);
      if (msg.type === "result" && msg.result) {
        if (msg.result.kind === "streamlines") setStream(msg.result);
        else if (msg.result.jobId === latest.current) setPaths(msg.result);
        if (msg.result.jobId === latest.current) setStatus({ state: "ready" });
      } else if (msg.type === "error" && msg.jobId === latest.current) {
        setStatus({ state: "error", message: msg.message ?? "computation failed" });
      }
      const run = pending.current;
      pending.current = null;
      run?.();
    };
    return () => {
      w.terminate();
      worker.current = null;
      busy.current = false;
      pending.current = null;
      runningKey.current = null;
      setStream(null);
      setPaths(null);
      setWorkerFrames([]);
    };
  }, [active, jobId, solid]);

  /** Sends a request now, or queues it (replacing any queued one); cancels the running one if asked. */
  const submit = (key: string, cancelRunning: boolean, post: (jobId: number) => WorkerRequest, statusFrame: number) => {
    const jobNo = nextJob.current++;
    latest.current = jobNo;
    const send = () => {
      if (latest.current !== jobNo || !worker.current) return;
      busy.current = true;
      runningKey.current = key;
      setStatus({ state: "computing", frame: statusFrame });
      worker.current.postMessage(post(jobNo));
    };
    if (busy.current) {
      pending.current = send;
      if (cancelRunning || runningKey.current !== key) worker.current?.postMessage({ type: "cancel", jobId: jobNo } satisfies WorkerRequest);
    } else {
      send();
    }
  };

  // Instantaneous streamlines of the displayed frame.
  useEffect(() => {
    if (settings.mode !== "streamlines" || !jobId || !geometry || !worker.current) return;
    if (!frameAvailable) {
      setStatus({ state: "unavailable", message: "Export u(t), v(t) and w(t) with the simulation to compute streamlines." });
      return;
    }
    const seeds =
      settings.seedMode === "slice" && slice
        ? sliceRakeSeeds(geometry, slice.axis, slice.coordinate, settings.density)
        : inletSeeds(geometry, settings.density);
    // Stagnation reference: the frame's largest velocity component (header statistics).
    const reference =
      Math.max(
        ...fields
          .filter((f) => f.frame.index === frame && (VELOCITY_COMPONENTS as readonly string[]).includes(f.name))
          .map((f) => f.stats.max_abs ?? 0),
      ) || 1;
    const options: TraceOptions = { ...DEFAULT_TRACE, direction: settings.direction, maxLength: settings.maxLength, referenceSpeed: reference };
    // Same key = same settings (only the frame differs): let the running one finish.
    const key = JSON.stringify(["streamlines", settings.seedMode, settings.density, settings.direction, settings.maxLength, slice]);
    submit(key, false, (jobId) => ({ type: "streamlines", request: { jobId, frame, seeds, options } }), frame);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [settings.mode, settings.seedMode, settings.density, settings.direction, settings.maxLength, jobId, geometry, frame, frameAvailable, slice?.axis, slice?.coordinate, workerEpoch]);

  // Particle paths from the release frame (not recomputed when the displayed frame changes).
  useEffect(() => {
    if (settings.mode !== "pathlines" || !jobId || !geometry || !worker.current) return;
    if (releaseFrames.length < 2) {
      setPaths(null);
      setStatus({ state: "unavailable", message: "Particle paths need u(t), v(t) and w(t) exported at two or more frames after the release frame." });
      return;
    }
    if (temporal?.level === "withhold") {
      setPaths(null);
      setStatus({
        state: "withheld",
        message:
          `Tracers would move up to ${temporal.cellsPerFrame.toFixed(1)} cells between exported frames (limit 5). ` +
          `The velocity between frames is not known, so particle paths are withheld. Export frames at least every ` +
          `${(temporal.intervalForWithhold ?? 0).toPrecision(2)} s (every ${(temporal.intervalForWarn ?? 0).toPrecision(2)} s to avoid the warning).`,
      });
      return;
    }
    const seeds =
      settings.seedMode === "slice" && slice
        ? sliceRakeSeeds(geometry, slice.axis, slice.coordinate, settings.density)
        : inletSeeds(geometry, settings.density);
    const key = JSON.stringify(["pathlines", settings.seedMode, settings.density, settings.releaseFrame, slice]);
    setPaths((p) => (p && p.releaseFrame === settings.releaseFrame ? p : null));
    submit(key, true, (jobId) => ({ type: "pathlines", request: { jobId, frames: releaseFrames, seeds, options: DEFAULT_PATH_OPTIONS } }), releaseFrames[0].index);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [settings.mode, settings.seedMode, settings.density, settings.releaseFrame, jobId, geometry, releaseFrames, temporal?.level, slice?.axis, slice?.coordinate, workerEpoch]);

  useEffect(() => {
    if (!active) setStatus({ state: "off" });
  }, [active]);

  return {
    streamlines: settings.mode === "streamlines" ? streamlinesForFrame(stream, frame) : null,
    /** Frame of the last finished streamline computation (it is hidden unless it is the displayed frame). */
    streamlineFrame: stream?.frame ?? null,
    lastStreamlines: settings.mode === "streamlines" ? stream : null,
    pathlines: settings.mode === "pathlines" ? paths : null,
    status,
    temporal,
    workerFrames,
  };
}

