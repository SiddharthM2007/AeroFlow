/**
 * Frame playback clock (Milestone 8). Pure functions over a small state so the timing rules can be
 * tested without timers:
 *
 * - The clock advances one frame per 1/fps seconds, and only once the next frame is decoded: frames
 *   are never skipped and the slice never blanks. If decoding is slower than the target rate, the
 *   achieved rate drops and is reported (stalled).
 * - At the last frame playback stops, or wraps to the first frame when looping.
 */

export const BASE_FPS = 4;
export const SPEEDS = [0.5, 1, 2, 4] as const;
export type Speed = (typeof SPEEDS)[number];

export interface PlaybackState {
  playing: boolean;
  position: number; // index into the frame list (0-based)
  count: number;
  speed: Speed;
  loop: boolean;
  lastAdvance: number; // ms timestamp of the last frame change while playing
  stalled: boolean; // the next frame was due but not yet decoded
  recent: number[]; // timestamps of recent advances (for the achieved rate)
}

export function initialPlayback(count: number): PlaybackState {
  return { playing: false, position: 0, count, speed: 1, loop: true, lastAdvance: 0, stalled: false, recent: [] };
}

export const targetFps = (speed: Speed): number => BASE_FPS * speed;

/** The position after `position` in playback order, or null at the end without looping. */
export function nextPosition(position: number, count: number, loop: boolean): number | null {
  if (count <= 1) return null;
  if (position + 1 < count) return position + 1;
  return loop ? 0 : null;
}

export function play(state: PlaybackState, now: number): PlaybackState {
  if (state.count <= 1) return state;
  // Pressing Play on the last frame without looping restarts from the first frame.
  const position = !state.loop && state.position >= state.count - 1 ? 0 : state.position;
  return { ...state, playing: true, position, lastAdvance: now, stalled: false, recent: [] };
}

export function pause(state: PlaybackState): PlaybackState {
  return { ...state, playing: false, stalled: false };
}

/** Advances when a frame is due and decoded. `ready(position)` says whether that frame is cached. */
export function tick(state: PlaybackState, now: number, ready: (position: number) => boolean): PlaybackState {
  if (!state.playing) return state;
  if (now - state.lastAdvance < 1000 / targetFps(state.speed)) return state;
  const next = nextPosition(state.position, state.count, state.loop);
  if (next === null) return { ...state, playing: false, stalled: false };
  if (!ready(next)) return state.stalled ? state : { ...state, stalled: true };
  const recent = [...state.recent, now].slice(-8);
  // Keep the cadence: the next frame is due one period after this one was due, so display-refresh
  // quantization does not lower the rate. After a stall, restart the cadence from now (never catch
  // up by skipping frames).
  const period = 1000 / targetFps(state.speed);
  const lastAdvance = now - state.lastAdvance < 2 * period ? state.lastAdvance + period : now;
  return { ...state, position: next, lastAdvance, stalled: false, recent };
}

/** Frames per second actually shown over the recent advances (null until there are two). */
export function achievedFps(state: PlaybackState): number | null {
  const r = state.recent;
  if (r.length < 2) return null;
  const span = r[r.length - 1] - r[0];
  return span > 0 ? ((r.length - 1) * 1000) / span : null;
}

/**
 * Positions to prefetch, in playback order. The look-ahead covers the time a decode takes at the
 * target rate (at least 2 frames, at most 8), and is limited so the prefetched frames use at most
 * half the cache budget.
 */
export function prefetchPositions(
  state: Pick<PlaybackState, "position" | "count" | "loop" | "speed">,
  decodeMs: number,
  frameBytes: number,
  budgetBytes: number,
): number[] {
  const byTime = Math.ceil((Math.max(decodeMs, 0) * targetFps(state.speed)) / 1000) + 1;
  const byMemory = frameBytes > 0 ? Math.floor(budgetBytes / 2 / frameBytes) : 8;
  const ahead = Math.max(0, Math.min(8, Math.max(2, byTime), byMemory));
  const out: number[] = [];
  let p = state.position;
  for (let n = 0; n < ahead; n++) {
    const q = nextPosition(p, state.count, state.loop);
    if (q === null || q === state.position) break;
    out.push(q);
    p = q;
  }
  return out;
}
