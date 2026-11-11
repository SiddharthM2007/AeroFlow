import { OrbitControls, TransformControls } from "@react-three/drei";
import { Canvas } from "@react-three/fiber";
import { useMemo, useState } from "react";
import { ArrowHelper, BoxGeometry, BufferGeometry, EdgesGeometry, MathUtils, Mesh, Vector3 } from "three";
import type { LoadedModel } from "../lib/loaders";
import type { Domain, GizmoMode, ModelTransform, TestBox } from "../lib/types";
import { GridOverlay, type GridOverlayProps } from "./GridOverlay";

interface ViewerProps {
  domain: Domain;
  model: LoadedModel | null;
  transform: ModelTransform;
  onTransformChange: (t: ModelTransform) => void;
  gizmoMode: GizmoMode;
  modelOpacity: number;
  testBox: TestBox | null;
  grid: GridOverlayProps;
}

export function Viewer(props: ViewerProps) {
  const { domain } = props;
  const center = useMemo(
    () => [domain.lx / 2, domain.ly / 2, domain.lz / 2] as [number, number, number],
    [domain],
  );
  const span = Math.max(domain.lx, domain.ly, domain.lz);

  return (
    <Canvas
      camera={{ position: [center[0] - span * 0.6, center[1] + span * 0.7, center[2] + span * 1.3], fov: 45, near: 0.001, far: span * 100 }}
      gl={{ antialias: true }}
    >
      <color attach="background" args={["#11151c"]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[5, 10, 7]} intensity={1.1} />
      <directionalLight position={[-6, -4, -5]} intensity={0.35} />

      <TunnelBox domain={domain} />
      {props.testBox && <TestBoxMesh box={props.testBox} />}
      {props.model && (
        <ModelObject
          model={props.model}
          transform={props.transform}
          onChange={props.onTransformChange}
          mode={props.gizmoMode}
          opacity={props.modelOpacity}
        />
      )}
      <GridOverlay {...props.grid} />

      <axesHelper args={[span * 0.15]} />
      <OrbitControls makeDefault target={center} />
    </Canvas>
  );
}

function TunnelBox({ domain }: { domain: Domain }) {
  const { lx, ly, lz } = domain;
  const edges = useMemo(() => new EdgesGeometry(new BoxGeometry(lx, ly, lz)), [lx, ly, lz]);
  const arrow = useMemo(() => {
    const len = Math.min(lx * 0.25, Math.max(ly, lz) * 0.5);
    return new ArrowHelper(new Vector3(1, 0, 0), new Vector3(-len * 1.2, ly / 2, lz / 2), len, 0x4fc3f7, len * 0.25, len * 0.12);
  }, [lx, ly, lz]);

  return (
    <group>
      <lineSegments geometry={edges} position={[lx / 2, ly / 2, lz / 2]}>
        <lineBasicMaterial color="#8fa3bf" />
      </lineSegments>
      {/* Floor of the tunnel, for spatial reference. */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[lx / 2, 0, lz / 2]}>
        <planeGeometry args={[lx, lz]} />
        <meshBasicMaterial color="#1c2430" transparent opacity={0.6} depthWrite={false} />
      </mesh>
      {/* Inlet plane at x = 0; flow is along +x. */}
      <mesh rotation={[0, Math.PI / 2, 0]} position={[0, ly / 2, lz / 2]}>
        <planeGeometry args={[lz, ly]} />
        <meshBasicMaterial color="#4fc3f7" transparent opacity={0.08} side={2} depthWrite={false} />
      </mesh>
      <primitive object={arrow} />
    </group>
  );
}

function TestBoxMesh({ box }: { box: TestBox }) {
  const size = box.max.map((hi, a) => hi - box.min[a]) as [number, number, number];
  const center = box.min.map((lo, a) => lo + size[a] / 2) as [number, number, number];
  const edges = useMemo(() => new EdgesGeometry(new BoxGeometry(...size)), [size[0], size[1], size[2]]);
  return (
    <lineSegments geometry={edges} position={center}>
      <lineBasicMaterial color="#ffb74d" />
    </lineSegments>
  );
}

interface ModelObjectProps {
  model: LoadedModel;
  transform: ModelTransform;
  onChange: (t: ModelTransform) => void;
  mode: GizmoMode;
  opacity: number;
}

function ModelObject({ model, transform, onChange, mode, opacity }: ModelObjectProps) {
  const [mesh, setMesh] = useState<Mesh<BufferGeometry> | null>(null);
  const rotation = transform.rotation.map((d) => MathUtils.degToRad(d)) as [number, number, number];

  return (
    <>
      <mesh ref={setMesh} geometry={model.geometry} position={transform.position} rotation={rotation} scale={transform.scale}>
        <meshStandardMaterial
          color="#cfd8dc"
          metalness={0.1}
          roughness={0.6}
          transparent={opacity < 1}
          opacity={opacity}
          depthWrite={opacity >= 1}
        />
      </mesh>
      {mesh && mode !== "none" && (
        <TransformControls
          object={mesh}
          mode={mode}
          size={0.8}
          onObjectChange={() =>
            onChange({
              position: mesh.position.toArray() as [number, number, number],
              rotation: [mesh.rotation.x, mesh.rotation.y, mesh.rotation.z].map((r) => MathUtils.radToDeg(r)) as [number, number, number],
              scale: mesh.scale.toArray() as [number, number, number],
            })
          }
        />
      )}
    </>
  );
}
