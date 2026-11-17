/// <reference lib="webworker" />
// Web Worker host for FlowEngine. The worker fetches and decodes the velocity frames itself
// (field files are HTTP-cached), so the main thread never copies or transfers velocity arrays.
import { decodeField } from "../fields/format";
import { FlowEngine, type PathlineRequest, type StreamlineRequest } from "./engine";
import { macVelocityFromFields } from "./velocityField";

export type WorkerRequest =
  | { type: "dataset"; key: string; apiJobId: string; solid: Uint8Array }
  | { type: "streamlines"; request: StreamlineRequest }
  | { type: "pathlines"; request: PathlineRequest }
  | { type: "cancel"; jobId: number };

const engine = new FlowEngine();

async function fetchComponent(apiJobId: string, name: string, frame: number, signal: AbortSignal) {
  const resp = await fetch(`/api/grid-jobs/${encodeURIComponent(apiJobId)}/fields/${name}?frame=${frame}`, { signal });
  if (!resp.ok) throw new Error(`could not load ${name} frame ${frame} (${resp.status})`);
  return decodeField(await resp.arrayBuffer());
}

self.onmessage = async (e: MessageEvent<WorkerRequest>) => {
  const msg = e.data;
  if (msg.type === "dataset") {
    const { apiJobId, solid } = msg;
    engine.configure(msg.key, async (frame, signal) => {
      const [u, v, w] = await Promise.all(["sim_u", "sim_v", "sim_w"].map((n) => fetchComponent(apiJobId, n, frame, signal)));
      return macVelocityFromFields(u, v, w, solid);
    });
    return;
  }
  if (msg.type === "cancel") {
    engine.announce(msg.jobId);
    return;
  }
  const jobId = msg.request.jobId;
  try {
    const result = msg.type === "streamlines" ? await engine.streamlines(msg.request) : await engine.pathlines(msg.request);
    if (result === null) {
      self.postMessage({ type: "cancelled", jobId });
      return;
    }
    const buffers: ArrayBuffer[] = [result.positions.buffer, result.speeds.buffer, result.offsets.buffer] as ArrayBuffer[];
    if (result.kind === "pathlines") buffers.push(result.times.buffer as ArrayBuffer);
    self.postMessage({ type: "result", result, cachedFrames: engine.cachedFrames() }, buffers);
  } catch (err) {
    self.postMessage({ type: "error", jobId, message: (err as Error).message });
  }
};
