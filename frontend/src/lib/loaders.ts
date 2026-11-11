import { BufferGeometry, Mesh, Vector3 } from "three";
import { OBJLoader } from "three/examples/jsm/loaders/OBJLoader.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { mergeGeometries } from "three/examples/jsm/utils/BufferGeometryUtils.js";

export interface LoadedModel {
  name: string;
  geometry: BufferGeometry;
  /** Bounding-box size in file units, before any transform. */
  size: Vector3;
  triangles: number;
}

/**
 * Parses an STL or OBJ file into a single BufferGeometry, recentred so its
 * bounding-box center is at the local origin (rotation/scale pivot about it).
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

  geometry.computeBoundingBox();
  const box = geometry.boundingBox!;
  const center = box.getCenter(new Vector3());
  geometry.translate(-center.x, -center.y, -center.z);
  geometry.computeVertexNormals();
  geometry.computeBoundingBox();

  return {
    name: file.name,
    geometry,
    size: box.getSize(new Vector3()),
    triangles: geometry.index ? geometry.index.count / 3 : position.count / 3,
  };
}
