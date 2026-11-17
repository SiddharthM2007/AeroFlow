import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { fetchField } from "../api";
import { decodeField, type FieldMetadata, type ScalarFieldData } from "../fields/format";
import { DEFAULT_CACHE_BUDGET_BYTES, FrameCache, isAbortError, type CacheStats } from "./frameCache";
import {
  achievedFps,
  initialPlayback,
  pause,
  play,
  prefetchPositions,
  tick,
  type PlaybackState,
  type Speed,
} from "./playback";

interface FrameKey {
  jobId: string;
  name: string;
  frame: number;
}

const keyOf = (k: FrameKey) => `${k.jobId}|${k.name}|${k.frame}`;
const sizeOf = (d: ScalarFieldData) => d.values.byteLength + d.valid.byteLength;

export interface FieldFrames {
  /** Decoded data for the selected frame; the previous frame of the same field stays shown while loading. */
  data: ScalarFieldData | null;
  loading: boolean;
  error: string | null;
  /** Frames of the selected field, by frame index. */
  frames: FieldMetadata[];
  position: number;
  playback: PlaybackState;
  achievedFps: number | null;
  cache: CacheStats;
  play: () => void;
  pause: () => void;
  step: (delta: number) => void;
  goTo: (position: number) => void;
  setSpeed: (speed: Speed) => void;
  setLoop: (loop: boolean) => void;
  /** Loads any exported field frame through the same cache (e.g. velocity components). */
  loadFrame: (name: string, frame: number) => Promise<ScalarFieldData>;
}

/**
 * Loads field frames through a bounded LRU cache, prefetches ahead of playback and runs the
 * playback clock (Milestone 8). `selectedFrame` is the frame index on screen; playback changes it
 * through `onFrameChange` only once the next frame is decoded.
 */
export function useFieldFrames(
  jobId: string | null,
  fields: FieldMetadata[],
  selectedField: string | null,
  selectedFrame: number,
  onFrameChange: (frame: number) => void,
  budgetBytes = DEFAULT_CACHE_BUDGET_BYTES,
  /** Other fields needed for each displayed frame (prefetched alongside, e.g. sim_u/v/w). */
  companions: string[] = [],
): FieldFrames {
  const decodeMs = useRef(0);
  const cache = useMemo(
    () =>
      new FrameCache<FrameKey, ScalarFieldData>(
        keyOf,
        async (k, signal) => {
          const buffer = await fetchField(k.jobId, k.name, k.frame, signal);
          const start = performance.now();
          const decoded = decodeField(buffer);
          decodeMs.current = 0.7 * decodeMs.current + 0.3 * (performance.now() - start);
          return decoded;
        },
        sizeOf,
        budgetBytes,
      ),
    [budgetBytes],
  );
  // A new job invalidates every cached frame.
  useEffect(() => () => cache.clear(), [cache]);
  useEffect(() => cache.clear(), [cache, jobId]);

  const companionsKey = companions.join(",");
  const frames = useMemo(
    () => fields.filter((f) => f.name === selectedField).sort((a, b) => a.frame.index - b.frame.index),
    [fields, selectedField],
  );
  const indices = useMemo(() => frames.map((f) => f.frame.index), [frames]);
  const position = Math.max(0, indices.indexOf(selectedFrame));

  const [data, setData] = useState<ScalarFieldData | null>(null);
  const [status, setStatus] = useState<{ loading: boolean; error: string | null }>({ loading: false, error: null });
  const [stats, setStats] = useState<CacheStats>(cache.stats());
  useEffect(() => cache.subscribe(() => setStats(cache.stats())), [cache]);
  const [pb, setPb] = useState<PlaybackState>(() => initialPlayback(0));
  const pbRef = useRef(pb);
  pbRef.current = { ...pb, position, count: indices.length };

  // Show the selected frame: from the cache at once, otherwise keep the previous frame of the same
  // field on screen until the new one is decoded (no blank slice between frames).
  useEffect(() => {
    if (!jobId || !selectedField) {
      setData(null);
      return;
    }
    const key = { jobId, name: selectedField, frame: selectedFrame };
    cache.pin(key);
    const hit = cache.get(key);
    let cancelled = false;
    if (hit) {
      setData(hit);
      setStatus({ loading: false, error: null });
    } else {
      setData((d) => (d && d.meta.name === selectedField ? d : null));
      setStatus({ loading: true, error: null });
      cache
        .load(key)
        .then((d) => {
          if (cancelled) return;
          setData(d);
          setStatus({ loading: false, error: null });
        })
        .catch((e) => {
          if (!cancelled && !isAbortError(e)) setStatus({ loading: false, error: (e as Error).message });
        })
        .finally(() => !cancelled && setStats(cache.stats()));
    }
    // Prefetch ahead in playback order, within the cache budget (companion fields count too).
    const names = [selectedField, ...companionsKey.split(",").filter((c) => c && c !== selectedField)];
    const frameBytes = hit ? sizeOf(hit) * names.length : 0;
    const ahead = prefetchPositions(pbRef.current, decodeMs.current, frameBytes, budgetBytes);
    cache.prefetch(ahead.flatMap((p) => names.map((name) => ({ jobId, name, frame: indices[p] }))));
    setStats(cache.stats());
    return () => {
      cancelled = true;
    };
  }, [cache, jobId, selectedField, selectedFrame, indices, budgetBytes, companionsKey]);

  // Playback loop: advances only to decoded frames; pauses when the tab is hidden.
  useEffect(() => {
    if (!pb.playing || !jobId || !selectedField) return;
    let id = 0;
    const loop = (now: number) => {
      const cur = pbRef.current;
      const next = tick(cur, now, (p) => cache.has({ jobId, name: selectedField, frame: indices[p] }));
      if (next !== cur) {
        setPb(next);
        if (next.position !== cur.position) onFrameChange(indices[next.position]);
      }
      if (next.playing) id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    const onHidden = () => document.hidden && setPb((s) => pause(s));
    document.addEventListener("visibilitychange", onHidden);
    return () => {
      cancelAnimationFrame(id);
      document.removeEventListener("visibilitychange", onHidden);
    };
  }, [pb.playing, cache, jobId, selectedField, indices, onFrameChange]);

  // Stop playing when the field or job changes.
  useEffect(() => setPb((s) => pause(s)), [jobId, selectedField]);

  const loadFrame = useCallback(
    (name: string, frame: number) => {
      if (!jobId) return Promise.reject(new Error("no job"));
      return cache.load({ jobId, name, frame });
    },
    [cache, jobId],
  );

  const goTo = useCallback(
    (p: number) => {
      if (indices.length === 0) return;
      const q = ((p % indices.length) + indices.length) % indices.length;
      onFrameChange(indices[q]);
    },
    [indices, onFrameChange],
  );

  return {
    data,
    loading: status.loading,
    error: status.error,
    frames,
    position,
    playback: { ...pb, position, count: indices.length },
    achievedFps: pb.playing ? achievedFps(pb) : null,
    cache: stats,
    play: () => {
      const next = play(pbRef.current, performance.now());
      setPb(next);
      if (next.position !== position && indices.length > 0) onFrameChange(indices[next.position]);
    },
    pause: () => setPb((s) => pause(s)),
    step: (delta: number) => {
      setPb((s) => pause(s));
      goTo(position + delta);
    },
    goTo: (p: number) => {
      setPb((s) => pause(s));
      goTo(p);
    },
    setSpeed: (speed: Speed) => setPb((s) => ({ ...s, speed })),
    setLoop: (loop: boolean) => setPb((s) => ({ ...s, loop })),
    loadFrame,
  };
}
