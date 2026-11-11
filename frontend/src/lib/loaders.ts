import { BufferGeometry, Mesh, Vector3 } from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";

export interface LoadedModel {
  name: string;
  /** Imported geometry; vertex positions are exactly as stored in the file (file units). */
  geometry: BufferGeometry;
  /** Raw bounding-box size in file units. */
  rawSize: Vector3;
  /** Raw bounding-box center in file units; used as the transform pivot. */
  pivot: [number, number, number];
  triangles: number;
}

/**
 * Parses an STL or OBJ file into a single BufferGeometry. Vertex positions are not modified;
 * placement is entirely described by the model transform (see transform.ts).
 */
export async function loadModelFile(file: File): Promise<LoadedModel> {
  const ext = file.name.split(".").pop()?.toLowerCase();
  let geometry: BufferGeometry;

  if (ext === "stl") {
    geometry = new STLLoader().parse(await file.arrayBuffer());
  } else if (ext === "obj") {
    const group = new OBJLoader().parse(await file.text());
    group.updateMatrixWorld(true);
    const parts: BufferGeometry[] = [];
    group.traverse((obj) => {
      if (obj instanceof Mesh) {
        let g = (obj.geometry as BufferGeometry).clone().applyMatrix4(obj.matrixWorld);
        if (g.index) g = g.toNonIndexed();
        // Keep positions only so meshes with differing attributes can be merged.
        const positionOnly = new BufferGeometry();
        positionOnly.setAttribute("position", g.getAttribute("position"));
        parts.push(positionOnly);
      }
    });
    if (parts.length === 0) throw new Error("OBJ file contains no mesh faces");
    const merged = mergeGeometries(parts);
    if (!merged) throw new Error("Failed to merge OBJ meshes");
    geometry = merged;
  } else {
    throw new Error(`Unsupported file type ".${ext ?? ""}"; expected .stl or .obj`);
  }

  const position = geometry.getAttribute("position");
  if (!position || position.count === 0) throw new Error("Model contains no geometry");

  geometry.computeVertexNormals();
  geometry.computeBoundingBox();
  const box = geometry.boundingBox!;

  return {
    name: file.name,
    geometry,
    rawSize: box.getSize(new Vector3()),
    pivot: box.getCenter(new Vector3()).toArray() as [number, number, number],
    triangles: geometry.index ? geometry.index.count / 3 : position.count / 3,
  };
}
