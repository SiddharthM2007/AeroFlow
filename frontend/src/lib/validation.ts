// Client-side checks mirroring the backend/engine rules, so problems are shown
// before a request is sent. The backend remains the authority.
import type { Domain, Resolution, TestBox } from "./types";

export const MAX_CELLS_PER_AXIS = 512;

export function validateGridConfig(
  domain: Domain,
  resolution: Resolution,
  testBox: TestBox | null,
): string[] {
  const errors: string[] = [];
  const lengths = [domain.lx, domain.ly, domain.lz];
  const counts = [resolution.nx, resolution.ny, resolution.nz];

  ["lx", "ly", "lz"].forEach((name, a) => {
    if (!Number.isFinite(lengths[a]) || lengths[a] <= 0) errors.push(`${name} must be > 0`);
  });
  ["nx", "ny", "nz"].forEach((name, a) => {
    const n = counts[a];
    if (!Number.isInteger(n) || n < 1 || n > MAX_CELLS_PER_AXIS) {
      errors.push(`${name} must be an integer between 1 and ${MAX_CELLS_PER_AXIS}`);
    }
  });

  if (testBox) {
    "xyz".split("").forEach((axis, a) => {
      const lo = testBox.min[a];
      const hi = testBox.max[a];
      if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
        errors.push(`test box ${axis} bounds must be numbers`);
      } else if (!(lo < hi)) {
        errors.push(`test box min.${axis} must be less than max.${axis}`);
      } else if (lo < 0 || hi > lengths[a]) {
        errors.push(`test box ${axis} extent must lie within [0, ${lengths[a]}]`);
      }
    });
  }
  return errors;
}
