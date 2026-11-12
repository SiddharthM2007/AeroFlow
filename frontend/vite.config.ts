import { defineConfig } from "vitest/config";
import react from "@vitejs/plugin-react";

const apiTarget = process.env.AEROFLOW_API_URL ?? "http://127.0.0.1:8000";

export default defineConfig({
  plugins: [react()],
  // three.js alone is ~1 MB minified; code-splitting is not worth it yet.
  build: { chunkSizeWarningLimit: 1600 },
  // AFFIELD test fixtures are imported with ?inline in unit tests.
  assetsInclude: ["**/*.affield"],
  server: {
    proxy: { "/api": { target: apiTarget, changeOrigin: true } },
  },
  test: {
    environment: "node",
  },
});
