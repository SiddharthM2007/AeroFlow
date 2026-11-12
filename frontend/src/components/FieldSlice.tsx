import { useEffect, useMemo } from "react";
import {
  BufferGeometry,
  DataTexture,
  DoubleSide,
  Float32BufferAttribute,
  NearestFilter,
  RGBAFormat,
  SRGBColorSpace,
  UnsignedByteType,
} from "three";
import { sliceToRGBA, type ColorRange, type ColormapKind } from "../lib/fields/colormap";
import type { ScalarFieldData } from "../lib/fields/format";
import { extractSlice, sliceCorners, type Axis } from "../lib/fields/sampling";

export interface FieldSliceProps {
  field: ScalarFieldData;
  axis: Axis;
  index: number;
  range: ColorRange;
  colormap: ColormapKind;
  opacity: number;
}

/**
 * One textured quad per slice: each texel is one field sample (nearest filtering, no
 * interpolation), invalid samples are transparent. Works for any sampling location.
 */
export function FieldSlice({ field, axis, index, range, colormap, opacity }: FieldSliceProps) {
  const slice = useMemo(() => extractSlice(field, axis, index), [field, axis, index]);

  const texture = useMemo(() => {
    const t = new DataTexture(sliceToRGBA(slice, range, colormap), slice.width, slice.height, RGBAFormat, UnsignedByteType);
    t.magFilter = NearestFilter;
    t.minFilter = NearestFilter;
    t.colorSpace = SRGBColorSpace;
    t.generateMipmaps = false;
    t.needsUpdate = true;
    return t;
  }, [slice, range.min, range.max, colormap]);
  useEffect(() => () => texture.dispose(), [texture]);

  const geometry = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute("position", new Float32BufferAttribute(sliceCorners(slice).flat(), 3));
    g.setAttribute("uv", new Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
    g.setIndex([0, 1, 2, 0, 2, 3]);
    return g;
  }, [slice]);
  useEffect(() => () => geometry.dispose(), [geometry]);

  return (
    <mesh geometry={geometry} renderOrder={2}>
      <meshBasicMaterial
        map={texture}
        transparent
        opacity={opacity}
        side={DoubleSide}
        depthWrite={false}
        alphaTest={0.01}
        toneMapped={false}
      />
    </mesh>
  );
}
