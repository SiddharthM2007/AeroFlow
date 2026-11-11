"""Reference implementation of the model-to-world transform (Milestone 2A).

    p_world = T(position) · R(rotation) · S(scale) · U(unit) · (p_raw − pivot)

R uses Euler XYZ in degrees with matrix R = Rx · Ry · Rz (the Three.js "XYZ" order used by the
frontend). The serialized matrix is 4×4 row-major and maps raw file coordinates to tunnel
coordinates in meters (origin at the tunnel corner, flow +x, y up). This module is the
specification the Milestone 2B C++ voxelizer must match.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

UNIT_TO_METERS: dict[str, float] = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254}

Mat3 = list[list[float]]


def _matmul3(a: Mat3, b: Mat3) -> Mat3:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotation_matrix(rotation_deg: Sequence[float]) -> Mat3:
    rx, ry, rz = (math.radians(d) for d in rotation_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    return _matmul3(_matmul3(Rx, Ry), Rz)


def compose_matrix(
    unit_scale: float,
    pivot: Sequence[float],
    scale: Sequence[float],
    rotation_deg: Sequence[float],
    position: Sequence[float],
) -> list[float]:
    """Row-major 4×4 matrix for T·R·S·U·Translate(−pivot)."""
    R = rotation_matrix(rotation_deg)
    # Linear part A = R · diag(scale · unit_scale)
    A = [[R[i][j] * scale[j] * unit_scale for j in range(3)] for i in range(3)]
    t = [position[i] - sum(A[i][j] * pivot[j] for j in range(3)) for i in range(3)]
    return [
        A[0][0], A[0][1], A[0][2], t[0],
        A[1][0], A[1][1], A[1][2], t[1],
        A[2][0], A[2][1], A[2][2], t[2],
        0.0, 0.0, 0.0, 1.0,
    ]  # fmt: skip


def apply_matrix(matrix: Sequence[float], p: Sequence[float]) -> tuple[float, float, float]:
    m = matrix
    return (
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    )
