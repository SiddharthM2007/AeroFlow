"""Stored STL/OBJ -> world-space triangle file for the engine (Milestone 2B).

The engine receives triangles already transformed to tunnel coordinates in meters, in the
binary format documented in engine/include/aeroflow/mesh.hpp:

    b"AFTRI1\\0\\0" | uint64 n | n x 9 float64 (v0xyz v1xyz v2xyz), little-endian

To match the Three.js viewer exactly:
  * Raw coordinates are rounded to float32 (Three stores positions in a Float32Array, and
    binary STL is float32 already).
  * OBJ faces are fan-triangulated from their first vertex, with 1-based and negative indices
    resolved like Three's OBJLoader; only `v` and `f` records contribute geometry.
  * World coordinates use the same row-major matrix the frontend sent and the same
    evaluation order as lib/transform.ts worldBounds(): m0*x + m1*y + m2*z + m3.

Parsing is deliberately not a second voxelizer: no classification happens here.
"""

from __future__ import annotations

import re
import struct
import sys
from array import array
from collections.abc import Sequence
from pathlib import Path

TRIANGLE_FILE_MAGIC = b"AFTRI1\0\0"

_ASCII_VERTEX = re.compile(rb"^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)", re.MULTILINE)


class MeshParseError(ValueError):
    """The stored model cannot be converted to triangles."""


def parse_model(data: bytes, fmt: str) -> array:
    """Raw triangle positions in file units: float32, 9 values per triangle."""
    if fmt == "stl":
        return parse_stl(data)
    if fmt == "obj":
        return parse_obj(data)
    raise MeshParseError(f"unsupported model format '{fmt}'")


def parse_stl(data: bytes) -> array:
    out = array("f")
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + 50 * count:
            for record in struct.iter_unpack("<12fH", memoryview(data)[84:]):
                out.extend(record[3:12])
            return out

    vertices = _ASCII_VERTEX.findall(data)
    if len(vertices) % 3 != 0:
        raise MeshParseError("ASCII STL has a facet without exactly three vertices")
    try:
        out.extend(float(c) for v in vertices for c in v)
    except ValueError as exc:
        raise MeshParseError(f"ASCII STL has a non-numeric vertex coordinate: {exc}") from exc
    return out


def parse_obj(data: bytes) -> array:
    positions: list[tuple[float, float, float]] = []
    out = array("f")
    for line_no, raw_line in enumerate(data.decode("utf-8", errors="replace").splitlines(), 1):
        line = raw_line.strip()
        if line.startswith("v "):
            parts = line.split()
            if len(parts) < 4:
                raise MeshParseError(f"OBJ line {line_no}: vertex needs three coordinates")
            try:
                positions.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError as exc:
                raise MeshParseError(f"OBJ line {line_no}: invalid vertex coordinate") from exc
        elif line.startswith("f "):
            corners = []
            for token in line.split()[1:]:
                try:
                    index = int(token.split("/")[0])
                except ValueError as exc:
                    raise MeshParseError(f"OBJ line {line_no}: invalid face index '{token}'") from exc
                resolved = index - 1 if index > 0 else len(positions) + index
                if index == 0 or not 0 <= resolved < len(positions):
                    raise MeshParseError(f"OBJ line {line_no}: face index {index} is out of range")
                corners.append(positions[resolved])
            for k in range(1, len(corners) - 1):  # fan triangulation, as Three.js does
                out.extend((*corners[0], *corners[k], *corners[k + 1]))
    return out


def transform_positions(raw: array, matrix: Sequence[float]) -> array:
    """Applies a row-major 4x4 affine matrix to flat xyz positions, in float64."""
    m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11 = matrix[:12]
    out = array("d", bytes(8 * len(raw)))
    for n in range(0, len(raw), 3):
        x, y, z = raw[n], raw[n + 1], raw[n + 2]
        out[n] = m0 * x + m1 * y + m2 * z + m3
        out[n + 1] = m4 * x + m5 * y + m6 * z + m7
        out[n + 2] = m8 * x + m9 * y + m10 * z + m11
    return out


def bounds(positions: Sequence[float]) -> tuple[tuple[float, ...], tuple[float, ...]]:
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def write_triangle_file(path: Path, world: array) -> None:
    if len(world) % 9 != 0:
        raise ValueError("world positions must contain 9 values per triangle")
    payload = array("d", world)
    if sys.byteorder != "little":
        payload.byteswap()
    with open(path, "wb") as f:
        f.write(TRIANGLE_FILE_MAGIC)
        f.write(struct.pack("<Q", len(world) // 9))
        payload.tofile(f)


def prepare_world_mesh(
    model_path: Path, fmt: str, matrix: Sequence[float], max_triangles: int, out_path: Path
) -> int:
    """Parses the stored model, transforms it to world meters and writes the engine file.

    Returns the triangle count. Raises MeshParseError for unusable or oversized models.
    """
    raw = parse_model(model_path.read_bytes(), fmt)
    count = len(raw) // 9
    if count == 0:
        raise MeshParseError("model contains no triangles")
    if count > max_triangles:
        raise MeshParseError(
            f"model has {count:,} triangles; the server limit is {max_triangles:,} "
            "(set AEROFLOW_MAX_TRIANGLES to raise it)"
        )
    write_triangle_file(out_path, transform_positions(raw, matrix))
    return count
