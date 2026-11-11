import { describe, expect, it } from "vitest";
import { loadModelFile } from "./loaders";

const ASCII_STL = `solid tri
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 2 0 0
  vertex 0 4 0
 endloop
endfacet
endsolid tri
`;

const OBJ = `v 0 0 0
v 2 0 0
v 2 4 0
v 0 4 6
o second
f 1 2 3
f 1 3 4
`;

describe("loadModelFile", () => {
  it("loads ASCII STL and recentres it", async () => {
    const model = await loadModelFile(new File([ASCII_STL], "tri.stl"));
    expect(model.triangles).toBe(1);
    expect(model.size.toArray()).toEqual([2, 4, 0]);
    const box = model.geometry.boundingBox!;
    expect(box.min.x + box.max.x).toBeCloseTo(0);
    expect(box.min.y + box.max.y).toBeCloseTo(0);
  });

  it("loads OBJ and merges meshes", async () => {
    const model = await loadModelFile(new File([OBJ], "quad.OBJ"));
    expect(model.triangles).toBe(2);
    expect(model.size.toArray()).toEqual([2, 4, 6]);
  });

  it("rejects other formats", async () => {
    await expect(loadModelFile(new File(["x"], "part.step"))).rejects.toThrow(/Unsupported/);
  });
});
