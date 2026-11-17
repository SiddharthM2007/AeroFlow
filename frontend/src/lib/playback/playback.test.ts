import { describe, expect, it } from "vitest";
import {
  BASE_FPS,
  SPEEDS,
  achievedFps,
  initialPlayback,
  nextPosition,
  pause,
  play,
  prefetchPositions,
  targetFps,
  tick,
  type PlaybackState,
} from "./playback";

const always = () => true;

describe("playback clock", () => {
  it("offers 0.5x, 1x, 2x and 4x of the base rate in frames per second", () => {
    expect(SPEEDS).toEqual([0.5, 1, 2, 4]);
    expect(SPEEDS.map(targetFps)).toEqual([0.5 * BASE_FPS, BASE_FPS, 2 * BASE_FPS, 4 * BASE_FPS]);
  });

  it("advances one frame per 1/fps seconds while playing", () => {
    let s = play(initialPlayback(5), 0);
    const period = 1000 / targetFps(1);
    s = tick(s, period - 1, always);
    expect(s.position).toBe(0);
    s = tick(s, period, always);
    expect(s.position).toBe(1);
    s = tick(s, period + 1, always);
    expect(s.position).toBe(1); // not due yet
    s = tick(s, 2 * period, always);
    expect(s.position).toBe(2);
  });

  it("waits for the next frame instead of skipping it, and reports the stall", () => {
    let s = play(initialPlayback(5), 0);
    const period = 1000 / targetFps(1);
    s = tick(s, period, () => false);
    expect(s.position).toBe(0);
    expect(s.stalled).toBe(true);
    s = tick(s, 3 * period, (p) => p === 1);
    expect(s.position).toBe(1); // one frame, never two
    expect(s.stalled).toBe(false);
  });

  it("loops or stops at the last frame", () => {
    expect(nextPosition(3, 4, true)).toBe(0);
    expect(nextPosition(3, 4, false)).toBeNull();
    expect(nextPosition(0, 1, true)).toBeNull();
    let s = { ...play(initialPlayback(2), 0), loop: false };
    s = tick(s, 1000, always);
    expect(s.position).toBe(1);
    s = tick(s, 2000, always);
    expect(s.playing).toBe(false);
    expect(s.position).toBe(1);
    // Play again from the end restarts at the first frame.
    expect(play(s, 3000).position).toBe(0);
  });

  it("does nothing when paused or for a single frame", () => {
    const s = pause(play(initialPlayback(4), 0));
    expect(tick(s, 10_000, always)).toBe(s);
    expect(play(initialPlayback(1), 0).playing).toBe(false);
  });

  it("keeps the target rate when ticks arrive at display-refresh times (60 Hz)", () => {
    let s: PlaybackState = { ...play(initialPlayback(1000), 0), speed: 4 };
    let advances = 0;
    for (let t = 1000 / 60; t <= 4000; t += 1000 / 60) {
      const n = tick(s, t, always);
      if (n.position !== s.position) advances++;
      s = n;
    }
    expect(advances).toBeGreaterThanOrEqual(63); // 16 fps for 4 s, not 15 (66.7 ms per frame)
    expect(advances).toBeLessThanOrEqual(64);
  });

  it("restarts the cadence after a stall instead of catching up", () => {
    let s = play(initialPlayback(10), 0);
    const period = 1000 / targetFps(1);
    s = tick(s, period, () => false); // due but not ready
    s = tick(s, 5 * period, always); // ready much later: one frame, cadence restarts
    expect(s.position).toBe(1);
    expect(s.lastAdvance).toBe(5 * period);
    s = tick(s, 5.5 * period, always);
    expect(s.position).toBe(1);
  });

  it("measures the achieved rate", () => {
    let s: PlaybackState = { ...play(initialPlayback(10), 0), speed: 2 };
    const period = 1000 / targetFps(2);
    for (let n = 1; n <= 4; n++) s = tick(s, n * period, always);
    expect(achievedFps(s)).toBeCloseTo(targetFps(2), 6);
  });
});

describe("prefetch window", () => {
  const base = { position: 0, count: 10, loop: true, speed: 1 as const };
  it("looks at least two frames ahead, more when decoding is slow", () => {
    expect(prefetchPositions(base, 0, 1, 1e9)).toEqual([1, 2]);
    expect(prefetchPositions(base, 1000, 1, 1e9)).toEqual([1, 2, 3, 4, 5]); // 1 s decode at 4 fps
    expect(prefetchPositions({ ...base, speed: 4 }, 10_000, 1, 1e9)).toHaveLength(8); // capped
  });

  it("wraps when looping and stops at the end otherwise", () => {
    expect(prefetchPositions({ ...base, position: 9 }, 0, 1, 1e9)).toEqual([0, 1]);
    expect(prefetchPositions({ ...base, position: 9, loop: false }, 0, 1, 1e9)).toEqual([]);
    expect(prefetchPositions({ ...base, count: 2 }, 1000, 1, 1e9)).toEqual([1]); // never the current frame
  });

  it("keeps prefetched frames within half of the cache budget", () => {
    const mb = 1024 * 1024;
    expect(prefetchPositions(base, 10_000, 30 * mb, 256 * mb)).toHaveLength(4); // 128 MB / 30 MB
    expect(prefetchPositions(base, 10_000, 200 * mb, 256 * mb)).toEqual([]); // too big to prefetch
  });
});
