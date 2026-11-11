import { describe, expect, it } from "vitest";
import { loadModelFile } from "./loaders";

const ASCII_STL = `solid tri
facet normal 0 0 1
 outer loop
  vertex 10 20 30
  vertex 12 20 30
  vertex 10 24 30
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
  it("loads ASCII STL without modifying vertex positions", async () => {
    const model = await loadModelFile(new File([ASCII_STL], "tri.stl"));
    expect(model.triangles).toBe(1);
    expect(Array.from(model.geometry.getAttribute("position").array)).toEqual([10, 20, 30, 12, 20, 30, 10, 24, 30]);
    expect(model.rawSize.toArray()).toEqual([2, 4, 0]);
    expect(model.pivot).toEqual([11, 22, 30]);
  });

  it("loads OBJ and merges meshes", async () => {
    const model = await loadModelFile(new File([OBJ], "quad.OBJ"));
    expect(model.triangles).toBe(2);
    expect(model.rawSize.toArray()).toEqual([2, 4, 6]);
    expect(model.pivot).toEqual([1, 2, 3]);
  });

  it("rejects other formats", async () => {
    await expect(loadModelFile(new File(["x"], "part.step"))).rejects.toThrow(/Unsupported/);
  });
});
