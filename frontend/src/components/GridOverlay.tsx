import { useLayoutEffect, useMemo, useRef } from "react";
import { BufferGeometry, Float32BufferAttribute, InstancedMesh, Object3D } from "three";
import {
  CELL_FLUID,
  CELL_SOLID,
  buildLattice,
  cellCentersOfType,
  latticeSegmentCount,
  type GridGeometry,
} from "../lib/grid";
import type { DisplayOptions } from "../lib/types";

/** Above this many segments the full 3D lattice is skipped (a slice still renders). */
export const MAX_LATTICE_SEGMENTS = 150_000;

export interface GridOverlayProps {
  /** Grid implied by the current UI settings (drives the lattice preview). */
  preview: GridGeometry;
  /** Grid + classification returned by the engine, if any. */
  result: { grid: GridGeometry; cells: Uint8Array } | null;
  display: DisplayOptions;
}

export function GridOverlay({ preview, result, display }: GridOverlayProps) {
  const slice = display.slice.enabled ? { axis: display.slice.axis, index: display.slice.index } : undefined;
  const latticeGrid = result?.grid ?? preview;

  const lattice = useMemo(() => {
    if (!display.showLattice) return null;
    if (!slice && latticeSegmentCount(latticeGrid.resolution) > MAX_LATTICE_SEGMENTS) return null;
    const sliceIndexValid =
      !slice || slice.index < [latticeGrid.resolution.nx, latticeGrid.resolution.ny, latticeGrid.resolution.nz][slice.axis];
    if (!sliceIndexValid) return null;
    const g = new BufferGeometry();
    g.setAttribute("position", new Float32BufferAttribute(buildLattice(latticeGrid, slice), 3));
    return g;
  }, [display.showLattice, latticeGrid, slice?.axis, slice?.index]);

  const solidCenters = useMemo(
    () => (result && display.showSolid ? cellCentersOfType(result.grid, result.cells, CELL_SOLID, slice) : null),
    [result, display.showSolid, slice?.axis, slice?.index],
  );
  const fluidCenters = useMemo(() => {
    if (!result || !display.showFluid) return null;
    const g = new BufferGeometry();
    g.setAttribute("position", new Float32BufferAttribute(cellCentersOfType(result.grid, result.cells, CELL_FLUID, slice), 3));
    return g;
  }, [result, display.showFluid, slice?.axis, slice?.index]);

  const spacing = (result?.grid ?? preview).spacing;
  const minSpacing = Math.min(...spacing);

  return (
    <group>
      {lattice && (
        <lineSegments geometry={lattice}>
          <lineBasicMaterial color="#5c6b80" transparent opacity={slice ? 0.7 : 0.18} depthWrite={false} />
        </lineSegments>
      )}
      {solidCenters && solidCenters.length > 0 && (
        <CellInstances centers={solidCenters} size={result!.grid.spacing} color="#ff7043" />
      )}
      {fluidCenters && (
        <points geometry={fluidCenters}>
          <pointsMaterial color="#4fc3f7" size={minSpacing * 0.25} sizeAttenuation transparent opacity={0.6} depthWrite={false} />
        </points>
      )}
    </group>
  );
}

function CellInstances({ centers, size, color }: { centers: Float32Array; size: [number, number, number]; color: string }) {
  const ref = useRef<InstancedMesh>(null);
  const count = centers.length / 3;

  useLayoutEffect(() => {
    const mesh = ref.current;
    if (!mesh) return;
    const dummy = new Object3D();
    for (let n = 0; n < count; n++) {
      dummy.position.set(centers[3 * n], centers[3 * n + 1], centers[3 * n + 2]);
      dummy.updateMatrix();
      mesh.setMatrixAt(n, dummy.matrix);
    }
    mesh.instanceMatrix.needsUpdate = true;
    mesh.computeBoundingSphere();
  }, [centers, count]);

  // Slightly shrink cells so neighbouring faces stay distinguishable.
  return (
    <instancedMesh key={count} ref={ref} args={[undefined, undefined, count]}>
      <boxGeometry args={[size[0] * 0.92, size[1] * 0.92, size[2] * 0.92]} />
      <meshStandardMaterial color={color} roughness={0.7} />
    </instancedMesh>
  );
}
