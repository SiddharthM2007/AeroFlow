import { OrbitControls, TransformControls } from "@react-three/drei";
import { Canvas, useThree } from "@react-three/fiber";
import { useEffect, useMemo, useState } from "react";
import { ArrowHelper, BoxGeometry, EdgesGeometry, Group, MathUtils, PerspectiveCamera, Vector3 } from "three";
import { frameBounds } from "../lib/camera";
import type { LoadedModel } from "../lib/loaders";
import { UNIT_TO_METERS, type Bounds, type LengthUnit } from "../lib/transform";
import type { Domain, GizmoMode, ModelTransform, TestBox } from "../lib/types";
import { GridOverlay, type GridOverlayProps } from "./GridOverlay";

interface ViewerProps {
  domain: Domain;
  model: LoadedModel | null;
  units: LengthUnit;
  transform: ModelTransform;
  onTransformChange: (t: ModelTransform) => void;
  gizmoMode: GizmoMode;
  modelOpacity: number;
  testBox: TestBox | null;
  grid: GridOverlayProps;
  /** Camera framing request; a new `id` re-frames the camera onto `bounds`. */
  frame: { id: number; bounds: Bounds };
}

const FOV = 45;

export function Viewer(props: ViewerProps) {
  const { domain } = props;
  const span = Math.max(domain.lx, domain.ly, domain.lz);

  return (
    <Canvas camera={{ fov: FOV }} gl={{ antialias: true }}>
      <CameraRig frame={props.frame} />
      <color attach="background" args={["#11151c"]} />
      <ambientLight intensity={0.5} />
      <directionalLight position={[5, 10, 7]} intensity={1.1} />
      <directionalLight position={[-6, -4, -5]} intensity={0.35} />

      <TunnelBox domain={domain} />
      {props.testBox && <TestBoxMesh box={props.testBox} />}
      {props.model && (
        <ModelObject
          model={props.model}
          units={props.units}
          transform={props.transform}
          onChange={props.onTransformChange}
          mode={props.gizmoMode}
          opacity={props.modelOpacity}
        />
      )}
      <GridOverlay {...props.grid} />

      <axesHelper args={[span * 0.15]} />
      <OrbitControls makeDefault />
    </Canvas>
  );
}

/** Applies framing requests to the camera and orbit controls. */
function CameraRig({ frame }: { frame: { id: number; bounds: Bounds } }) {
  const camera = useThree((s) => s.camera) as PerspectiveCamera;
  const controls = useThree((s) => s.controls) as unknown as { target: Vector3; update: () => void } | null;
  const size = useThree((s) => s.size);

  useEffect(() => {
    const f = frameBounds(frame.bounds, FOV, size.width / Math.max(size.height, 1));
    camera.position.set(...f.position);
    camera.near = f.near;
    camera.far = f.far;
    camera.updateProjectionMatrix();
    if (controls) {
      controls.target.set(...f.target);
      controls.update();
    } else {
      camera.lookAt(...f.target);
    }
    // Re-frame only on explicit requests (and once controls exist), not on every resize.
  }, [frame.id, controls]);

  return null;
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
  units: LengthUnit;
  transform: ModelTransform;
  onChange: (t: ModelTransform) => void;
  mode: GizmoMode;
  opacity: number;
}

/**
 * Outer group = user transform (T·R·S, driven by the gizmo); inner mesh = unit conversion and
 * pivot offset (U·(p − pivot)). Together: p_world = T·R·S·U·(p_raw − pivot), matching
 * lib/transform.ts modelMatrix(). The imported geometry itself is never modified.
 */
function ModelObject({ model, units, transform, onChange, mode, opacity }: ModelObjectProps) {
  const [group, setGroup] = useState<Group | null>(null);
  const rotation = transform.rotation.map((d) => MathUtils.degToRad(d)) as [number, number, number];
  const u = UNIT_TO_METERS[units];
  const pivotOffset = model.pivot.map((c) => -c * u) as [number, number, number];

  return (
    <>
      <group ref={setGroup} position={transform.position} rotation={rotation} scale={transform.scale}>
        <mesh geometry={model.geometry} position={pivotOffset} scale={[u, u, u]}>
          <meshStandardMaterial
            color="#cfd8dc"
            metalness={0.1}
            roughness={0.6}
            transparent={opacity < 1}
            opacity={opacity}
            depthWrite={opacity >= 1}
          />
        </mesh>
      </group>
      {group && mode !== "none" && (
        <TransformControls
          object={group}
          mode={mode}
          size={0.8}
          onObjectChange={() =>
            onChange({
              position: group.position.toArray() as [number, number, number],
              rotation: [group.rotation.x, group.rotation.y, group.rotation.z].map((r) => MathUtils.radToDeg(r)) as [number, number, number],
              scale: group.scale.toArray() as [number, number, number],
            })
          }
        />
      )}
    </>
  );
}
