"""Validation and on-disk storage of uploaded STL/OBJ models."""

from __future__ import annotations

import json
import re
import struct
import uuid
from pathlib import Path

from .schemas import ModelInfo

ALLOWED_EXTENSIONS = {".stl": "stl", ".obj": "obj"}
_MODEL_ID_RE = re.compile(r"^[0-9a-f]{32}$")


class InvalidModel(ValueError):
    """The uploaded file is not an acceptable STL/OBJ model."""


def inspect_model(filename: str, data: bytes) -> ModelInfo:
    """Validate the file's type and structure. Raises InvalidModel with a user-facing message."""
    ext = Path(filename).suffix.lower()
    fmt = ALLOWED_EXTENSIONS.get(ext)
    if fmt is None:
        raise InvalidModel(
            f"unsupported file type '{ext or '(none)'}'; expected one of: .stl, .obj"
        )
    if not data:
        raise InvalidModel("file is empty")

    common = {"model_id": "", "filename": filename, "size_bytes": len(data)}
    if fmt == "stl":
        return ModelInfo(format="stl", **common, **_inspect_stl(data))
    return ModelInfo(format="obj", **common, **_inspect_obj(data))


def _inspect_stl(data: bytes) -> dict:
    # Binary STL: 80-byte header, uint32 triangle count, 50 bytes per triangle.
    # Checked first because binary headers may legitimately begin with "solid".
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + 50 * count:
            if count == 0:
                raise InvalidModel("binary STL contains no triangles")
            return {"encoding": "binary", "triangle_count": count}
        if b"\x00" in data[:4096]:
            raise InvalidModel(
                f"file is not a valid STL: binary header declares {count} triangles "
                f"({84 + 50 * count} bytes) but the file is {len(data)} bytes"
            )

    text = data.decode("ascii", errors="ignore")
    if text.lstrip().lower().startswith("solid"):
        facets = len(re.findall(r"\bendfacet\b", text))
        if facets == 0:
            raise InvalidModel("ASCII STL contains no facets")
        return {"encoding": "ascii", "triangle_count": facets}

    raise InvalidModel(
        "file is not a valid STL: binary size does not match its triangle count "
        "and it does not start with 'solid'"
    )


def _inspect_obj(data: bytes) -> dict:
    if b"\x00" in data[:4096]:
        raise InvalidModel("OBJ files must be text; file appears to be binary")
    vertices = faces = 0
    for line in data.decode("utf-8", errors="replace").splitlines():
        token = line.lstrip()[:2]
        if token == "v ":
            vertices += 1
        elif token == "f ":
            faces += 1
    if vertices == 0:
        raise InvalidModel("OBJ contains no vertices ('v' lines)")
    if faces == 0:
        raise InvalidModel("OBJ contains no faces ('f' lines)")
    return {"encoding": "ascii", "vertex_count": vertices, "face_count": faces}


class ModelStore:
    def __init__(self, root: Path) -> None:
        self.root = root

    def save(self, info: ModelInfo, data: bytes) -> ModelInfo:
        self.root.mkdir(parents=True, exist_ok=True)
        model_id = uuid.uuid4().hex
        stored = info.model_copy(update={"model_id": model_id})
        # Stored under a generated name; the client filename is metadata only.
        (self.root / f"{model_id}.{info.format}").write_bytes(data)
        (self.root / f"{model_id}.json").write_text(stored.model_dump_json())
        return stored

    def raw_path(self, model_id: str) -> Path | None:
        """Path of the stored model file (named by its generated id, never the client name)."""
        info = self.get(model_id)
        if info is None:
            return None
        path = self.root / f"{model_id}.{info.format}"
        return path if path.is_file() else None

    def get(self, model_id: str) -> ModelInfo | None:
        if not _MODEL_ID_RE.match(model_id):
            return None
        meta = self.root / f"{model_id}.json"
        if not meta.is_file():
            return None
        return ModelInfo.model_validate(json.loads(meta.read_text()))
