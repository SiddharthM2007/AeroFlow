/**
 * Bounded LRU cache of decoded field frames (Milestone 8).
 *
 * - Entries are charged by their decoded size (values + validity mask); the least recently used
 *   entries are evicted once the total exceeds the budget. The pinned entry (the frame on screen)
 *   is never evicted; a single frame larger than the whole budget is still kept while pinned.
 * - Concurrent requests for the same frame share one load.
 * - Prefetches can be superseded: prefetch() aborts earlier prefetches that are no longer wanted.
 */

export const DEFAULT_CACHE_BUDGET_BYTES = 256 * 1024 * 1024;

export interface CacheStats {
  entries: number;
  bytes: number;
  budget: number;
  hits: number;
  misses: number;
  evictions: number;
}

interface Pending<V> {
  promise: Promise<V>;
  controller: AbortController;
  prefetch: boolean;
}

export class FrameCache<K, V> {
  private readonly entries = new Map<string, { value: V; bytes: number }>(); // insertion order = LRU order
  private readonly pending = new Map<string, Pending<V>>();
  private bytes = 0;
  private pinned: string | null = null;
  private hits = 0;
  private misses = 0;
  private evictions = 0;
  private readonly listeners = new Set<() => void>();

  constructor(
    private readonly keyOf: (key: K) => string,
    private readonly loader: (key: K, signal: AbortSignal) => Promise<V>,
    private readonly sizeOf: (value: V) => number,
    private readonly budget = DEFAULT_CACHE_BUDGET_BYTES,
  ) {}

  /** The cached value, refreshed as most recently used; undefined on a miss. */
  get(key: K): V | undefined {
    const k = this.keyOf(key);
    const e = this.entries.get(k);
    if (!e) return undefined;
    this.entries.delete(k);
    this.entries.set(k, e);
    return e.value;
  }

  has(key: K): boolean {
    return this.entries.has(this.keyOf(key));
  }

  /** Marks the frame on screen; it is never evicted while pinned. */
  pin(key: K | null): void {
    this.pinned = key === null ? null : this.keyOf(key);
  }

  /** Loads (or returns) a frame for display. A pending prefetch of the same frame is promoted. */
  load(key: K): Promise<V> {
    const k = this.keyOf(key);
    const hit = this.get(key);
    if (hit !== undefined) {
      this.hits++;
      return Promise.resolve(hit);
    }
    const p = this.pending.get(k);
    if (p) {
      p.prefetch = false;
      return p.promise;
    }
    this.misses++;
    return this.start(key, k, false);
  }

  /**
   * Requests the given frames in the background (in order) and aborts earlier prefetches that are
   * not in the list. Already cached or loading frames are left alone.
   */
  prefetch(keys: K[]): void {
    const wanted = new Set(keys.map(this.keyOf));
    for (const [k, p] of this.pending) {
      if (p.prefetch && !wanted.has(k)) {
        p.controller.abort();
        this.pending.delete(k);
      }
    }
    for (const key of keys) {
      const k = this.keyOf(key);
      if (this.entries.has(k) || this.pending.has(k)) continue;
      this.start(key, k, true).catch(() => undefined); // failures surface when the frame is shown
    }
  }

  /** Drops everything (e.g. when the job changes) and aborts pending loads. */
  clear(): void {
    for (const p of this.pending.values()) p.controller.abort();
    this.pending.clear();
    this.entries.clear();
    this.bytes = 0;
    this.pinned = null;
    this.notify();
  }

  private notify(): void {
    for (const l of this.listeners) l();
  }

  /** Called whenever entries are added, evicted or cleared (for status displays). */
  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  stats(): CacheStats {
    return {
      entries: this.entries.size,
      bytes: this.bytes,
      budget: this.budget,
      hits: this.hits,
      misses: this.misses,
      evictions: this.evictions,
    };
  }

  private start(key: K, k: string, prefetch: boolean): Promise<V> {
    const controller = new AbortController();
    const promise = this.loader(key, controller.signal).then(
      (value) => {
        if (this.pending.get(k)?.promise === promise) this.pending.delete(k);
        if (controller.signal.aborted) throw new DOMException("frame load aborted", "AbortError");
        this.insert(k, value);
        return value;
      },
      (err: unknown) => {
        if (this.pending.get(k)?.promise === promise) this.pending.delete(k);
        throw err;
      },
    );
    this.pending.set(k, { promise, controller, prefetch });
    return promise;
  }

  private insert(k: string, value: V): void {
    const old = this.entries.get(k);
    if (old) {
      this.bytes -= old.bytes;
      this.entries.delete(k);
    }
    const bytes = this.sizeOf(value);
    this.entries.set(k, { value, bytes });
    this.bytes += bytes;
    // Evict least recently used entries (never the pinned one or the one just inserted).
    for (const [key, e] of this.entries) {
      if (this.bytes <= this.budget) break;
      if (key === this.pinned || key === k) continue;
      this.entries.delete(key);
      this.bytes -= e.bytes;
      this.evictions++;
    }
    this.notify();
  }
}

export const isAbortError = (e: unknown): boolean => e instanceof DOMException && e.name === "AbortError";
