"""AFFIELD v1 numerical field files: validation and per-job storage (Milestone 4A).

Format (see engine/include/aeroflow/field_io.hpp):

    bytes 0..7    b"AFFIELD\\0"
    bytes 8..11   uint32 LE version (1)
    bytes 12..15  uint32 LE JSON header length H
    bytes 16..    UTF-8 JSON header (space-padded; blocks start 8-byte aligned)
    blocks        "values" float32 LE and "valid" uint8, x-fastest over header["shape"]

The backend never re-encodes field data: it validates the engine's file and serves the same
bytes. Files live in data/jobs/<job_id>/ for as long as the job is retained in memory.
"""

from __future__ import annotations

import json
import shutil
import struct
from pathlib import Path
from typing import Any

FIELD_MAGIC = b"AFFIELD\0"
FIELD_VERSION = 1
FIELD_MEDIA_TYPE = "application/octet-stream"
KNOWN_FIELDS = ("divergence_diagnostic",)

_DTYPE_SIZE = {"float32": 4, "uint8": 1}
_REQUIRED_BLOCKS = {"values": "float32", "valid": "uint8"}
_LOCATION_EXTRA = {"cell": (0, 0, 0), "face_x": (1, 0, 0), "face_y": (0, 1, 0), "face_z": (0, 0, 1)}
_HEADER_SLACK = 64 * 1024  # generous allowance for the JSON header when estimating sizes


class FieldFileError(ValueError):
    """A field file is missing, truncated or inconsistent with its header or grid."""


def field_filename(name: str, frame: int) -> str:
    return f"{name}.f{frame}.affield"


def estimate_field_bytes(cell_count: int) -> int:
    """Upper-bound size of one cell-centered scalar field file (float32 values + uint8 valid)."""
    return cell_count * (4 + 1) + 8 + _HEADER_SLACK


def expected_shape(location: str, resolution: dict[str, int]) -> list[int]:
    extra = _LOCATION_EXTRA.get(location)
    if extra is None:
        raise FieldFileError(f"unknown field location '{location}'")
    return [resolution["nx"] + extra[0], resolution["ny"] + extra[1], resolution["nz"] + extra[2]]


def parse_field_header(data: bytes, resolution: dict[str, int] | None = None) -> dict[str, Any]:
    """Validates an AFFIELD file's bytes and returns its header. Raises FieldFileError."""
    if len(data) < 16 or data[:8] != FIELD_MAGIC:
        raise FieldFileError("not an AFFIELD file (bad magic)")
    version, header_len = struct.unpack_from("<II", data, 8)
    if version != FIELD_VERSION:
        raise FieldFileError(f"unsupported AFFIELD version {version} (expected {FIELD_VERSION})")
    if 16 + header_len > len(data):
        raise FieldFileError("AFFIELD header is truncated")
    try:
        header = json.loads(data[16 : 16 + header_len].decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise FieldFileError(f"AFFIELD header is not valid JSON: {exc}") from exc

    if header.get("format") != "aeroflow-field" or header.get("version") != FIELD_VERSION:
        raise FieldFileError("AFFIELD header has the wrong format or version")
    if header.get("byte_order") != "little" or header.get("index_order") != "i + n0*(j + n1*k)":
        raise FieldFileError("AFFIELD header has an unsupported byte or index order")
    shape = header.get("shape")
    if not (isinstance(shape, list) and len(shape) == 3 and all(isinstance(n, int) and n > 0 for n in shape)):
        raise FieldFileError("AFFIELD header has an invalid shape")
    grid_res = header.get("grid", {}).get("resolution", {})
    if expected_shape(header.get("location", ""), grid_res) != shape:
        raise FieldFileError("AFFIELD shape does not match its location on the grid")
    if resolution is not None and grid_res != resolution:
        raise FieldFileError(f"AFFIELD grid {grid_res} does not match the job grid {resolution}")

    count = shape[0] * shape[1] * shape[2]
    blocks = {b.get("name"): b for b in header.get("blocks", [])}
    for name, dtype in _REQUIRED_BLOCKS.items():
        block = blocks.get(name)
        if block is None:
            raise FieldFileError(f"AFFIELD file has no '{name}' block")
        offset, length = block.get("offset"), block.get("length")
        if block.get("dtype") != dtype or length != count * _DTYPE_SIZE[dtype]:
            raise FieldFileError(f"AFFIELD '{name}' block has the wrong type or length for shape {shape}")
        if not isinstance(offset, int) or offset % 8 or offset < 16 + header_len or offset + length > len(data):
            raise FieldFileError(f"AFFIELD '{name}' block lies outside the file")
    return header


class FieldStore:
    """Per-job field files under `root`, bounded by a total disk budget."""

    def __init__(self, root: Path, max_total_bytes: int) -> None:
        self.root = root
        self.max_total_bytes = max_total_bytes

    def reset(self) -> None:
        """Removes all stored fields (jobs are in-memory, so files from a previous run are orphans)."""
        shutil.rmtree(self.root, ignore_errors=True)
        self.root.mkdir(parents=True, exist_ok=True)

    def job_dir(self, job_id: str) -> Path:
        path = self.root / job_id
        path.mkdir(parents=True, exist_ok=True)
        return path

    def total_bytes(self) -> int:
        return sum(p.stat().st_size for p in self.root.glob("*/*.affield") if p.is_file())

    def has_room_for(self, extra_bytes: int) -> bool:
        return self.total_bytes() + extra_bytes <= self.max_total_bytes

    def register(self, job_id: str, summaries: list[dict], resolution: dict[str, int], max_field_bytes: int) -> list[dict]:
        """Validates the engine's exported files; returns their headers (without block offsets)."""
        headers = []
        for summary in summaries:
            name, frame = summary.get("name"), summary.get("frame", {}).get("index")
            if name not in KNOWN_FIELDS or not isinstance(frame, int):
                raise FieldFileError(f"engine reported an unexpected field {summary!r}")
            path = self.root / job_id / field_filename(name, frame)
            if not path.is_file():
                raise FieldFileError(f"field '{name}' frame {frame} was not written")
            if path.stat().st_size > max_field_bytes:
                raise FieldFileError(f"field '{name}' is {path.stat().st_size:,} bytes; limit is {max_field_bytes:,}")
            header = parse_field_header(path.read_bytes(), resolution)
            headers.append({k: v for k, v in header.items() if k != "blocks"} | {"bytes": path.stat().st_size})
        return headers

    def path(self, job_id: str, name: str, frame: int) -> Path | None:
        path = self.root / job_id / field_filename(name, frame)
        return path if path.is_file() else None

    def remove(self, job_id: str) -> None:
        shutil.rmtree(self.root / job_id, ignore_errors=True)
