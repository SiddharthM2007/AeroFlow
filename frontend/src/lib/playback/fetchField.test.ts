import { afterEach, describe, expect, it, vi } from "vitest";
import { ApiError, fetchField } from "../api";
import { isAbortError } from "./frameCache";

afterEach(() => vi.unstubAllGlobals());

describe("fetchField for cached playback", () => {
  it("requests the frame and passes the abort signal through", async () => {
    const fetchMock = vi.fn(async () => new Response(new Uint8Array([1, 2, 3]), { status: 200 }));
    vi.stubGlobal("fetch", fetchMock);
    const controller = new AbortController();
    const buf = await fetchField("job1", "sim_pressure", 3, controller.signal);
    expect(new Uint8Array(buf)).toEqual(new Uint8Array([1, 2, 3]));
    expect(fetchMock).toHaveBeenCalledWith("/api/grid-jobs/job1/fields/sim_pressure?frame=3", { signal: controller.signal });
  });

  it("reports a cancelled prefetch as an abort, not as an unreachable backend", async () => {
    const controller = new AbortController();
    vi.stubGlobal("fetch", vi.fn(async () => {
      controller.abort();
      throw new TypeError("aborted");
    }));
    const err = await fetchField("job1", "sim_u", 1, controller.signal).catch((e: unknown) => e);
    expect(isAbortError(err)).toBe(true);
  });

  it("still reports network failures and missing frames", async () => {
    vi.stubGlobal("fetch", vi.fn(async () => {
      throw new TypeError("network");
    }));
    await expect(fetchField("job1", "sim_u", 1)).rejects.toBeInstanceOf(ApiError);
    vi.stubGlobal("fetch", vi.fn(async () => new Response(JSON.stringify({ detail: "no frame 9" }), { status: 404 })));
    await expect(fetchField("job1", "sim_u", 9)).rejects.toThrow("no frame 9");
  });
});
