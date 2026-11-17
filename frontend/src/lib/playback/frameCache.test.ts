import { describe, expect, it } from "vitest";
import { FrameCache, isAbortError } from "./frameCache";

/** A controllable loader: each requested key resolves when release(key) is called. */
function harness(budget: number, size = (v: number) => v) {
  const waiting = new Map<number, { resolve: (v: number) => void; signal: AbortSignal }>();
  const calls: number[] = [];
  const cache = new FrameCache<number, number>(
    (k) => String(k),
    (k, signal) =>
      new Promise<number>((resolve) => {
        calls.push(k);
        waiting.set(k, { resolve, signal });
      }),
    size,
    budget,
  );
  const release = async (k: number, value = 10) => {
    waiting.get(k)!.resolve(value);
    waiting.delete(k);
    await Promise.resolve();
    await Promise.resolve();
  };
  return { cache, calls, waiting, release };
}

describe("FrameCache", () => {
  it("caches decoded frames and counts hits and misses", async () => {
    const { cache, calls, release } = harness(100);
    const p = cache.load(1);
    await release(1, 10);
    expect(await p).toBe(10);
    expect(await cache.load(1)).toBe(10);
    expect(calls).toEqual([1]);
    expect(cache.stats()).toMatchObject({ entries: 1, bytes: 10, hits: 1, misses: 1 });
  });

  it("shares one load between concurrent requests", async () => {
    const { cache, calls, release } = harness(100);
    const a = cache.load(2);
    const b = cache.load(2);
    await release(2, 7);
    expect(await a).toBe(7);
    expect(await b).toBe(7);
    expect(calls).toEqual([2]);
  });

  it("evicts least recently used frames beyond the budget", async () => {
    const { cache, release } = harness(30);
    for (const k of [1, 2, 3]) {
      const p = cache.load(k);
      await release(k, 10);
      await p;
    }
    cache.get(1); // 1 is now more recent than 2
    const p = cache.load(4);
    await release(4, 10);
    await p;
    expect(cache.has(2)).toBe(false); // least recently used
    expect([1, 3, 4].every((k) => cache.has(k))).toBe(true);
    expect(cache.stats()).toMatchObject({ bytes: 30, evictions: 1 });
  });

  it("never evicts the pinned frame, even when one frame exceeds the budget", async () => {
    const { cache, release } = harness(25);
    const a = cache.load(1);
    await release(1, 20);
    await a;
    cache.pin(1);
    const b = cache.load(2);
    await release(2, 20);
    await b;
    expect(cache.has(1)).toBe(true); // pinned
    expect(cache.has(2)).toBe(true); // just inserted
    const c = cache.load(3);
    await release(3, 20);
    await c;
    expect(cache.has(1)).toBe(true);
    expect(cache.has(2)).toBe(false);
    expect(cache.stats().bytes).toBe(40); // over budget only by the pinned frame
  });

  it("aborts prefetches that are no longer wanted and keeps the rest", async () => {
    const { cache, calls, waiting } = harness(1000);
    cache.prefetch([1, 2, 3]);
    expect(calls).toEqual([1, 2, 3]);
    const signal1 = waiting.get(1)!.signal;
    cache.prefetch([2, 3, 4]);
    expect(signal1.aborted).toBe(true);
    expect(waiting.get(2)!.signal.aborted).toBe(false);
    expect(calls).toEqual([1, 2, 3, 4]); // 2 and 3 are not requested twice
  });

  it("promotes a pending prefetch when the frame is needed, so it is not aborted", async () => {
    const { cache, waiting, release } = harness(1000);
    cache.prefetch([5]);
    const shown = cache.load(5);
    cache.prefetch([]); // would abort plain prefetches
    expect(waiting.get(5)!.signal.aborted).toBe(false);
    await release(5, 3);
    expect(await shown).toBe(3);
  });

  it("an aborted load is reported as an abort, not cached", async () => {
    const { cache, release } = harness(1000);
    cache.prefetch([6]);
    cache.clear();
    await release(6, 1).catch(() => undefined);
    expect(cache.has(6)).toBe(false);
    const e = new DOMException("x", "AbortError");
    expect(isAbortError(e)).toBe(true);
    expect(isAbortError(new Error("x"))).toBe(false);
  });

  it("clear() drops everything", async () => {
    const { cache, release } = harness(100);
    const p = cache.load(1);
    await release(1, 10);
    await p;
    cache.clear();
    expect(cache.stats()).toMatchObject({ entries: 0, bytes: 0 });
  });
});
