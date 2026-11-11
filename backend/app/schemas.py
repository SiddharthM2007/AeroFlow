"""Request/response models. Field validation mirrors the engine's config checks so
clients get precise 422 errors before the engine is ever invoked."""

from __future__ import annotations

from datetime import datetime
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

Length = Annotated[float, Field(gt=0, allow_inf_nan=False, description="Length in meters")]
Coordinate = Annotated[float, Field(allow_inf_nan=False)]
Point3 = Annotated[
    list[Coordinate], Field(min_length=3, max_length=3, description="[x, y, z] in meters")
]
CellCount = Annotated[int, Field(ge=1, strict=True)]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class Domain(StrictModel):
    lx: Length
    ly: Length
    lz: Length


class Resolution(StrictModel):
    nx: CellCount
    ny: CellCount
    nz: CellCount

    @property
    def total(self) -> int:
        return self.nx * self.ny * self.nz


class TestBox(StrictModel):
    __test__ = False  # not a pytest test class

    min: Point3
    max: Point3


class GridJobRequest(StrictModel):
    domain: Domain
    resolution: Resolution
    test_box: TestBox | None = None
    model_id: str | None = Field(
        default=None,
        description="Uploaded model to associate with the job. Recorded only; "
        "geometry is not voxelized in Milestone 1.",
    )
    include_cells: bool = Field(
        default=True, description="Return per-cell classification (base64 uint8)."
    )

    @model_validator(mode="after")
    def _box_inside_domain(self) -> GridJobRequest:
        if self.test_box is None:
            return self
        lengths = (self.domain.lx, self.domain.ly, self.domain.lz)
        for axis, lo, hi, length in zip("xyz", self.test_box.min, self.test_box.max, lengths):
            if not lo < hi:
                raise ValueError(f"test_box: min.{axis} must be less than max.{axis}")
            if lo < 0 or hi > length:
                raise ValueError(
                    f"test_box: extent along {axis} must lie within the domain [0, {length}]"
                )
        return self

    def engine_config(self) -> dict[str, Any]:
        """The JSON document passed to the aeroflow-grid executable."""
        return self.model_dump(include={"domain", "resolution", "test_box"}, mode="json")


class ModelInfo(BaseModel):
    model_id: str
    filename: str
    format: Literal["stl", "obj"]
    encoding: Literal["binary", "ascii"]
    size_bytes: int
    triangle_count: int | None = None
    vertex_count: int | None = None
    face_count: int | None = None


class GridJob(BaseModel):
    job_id: str
    status: Literal["completed", "failed"]
    created_at: datetime
    finished_at: datetime
    request: GridJobRequest
    result: dict[str, Any] | None = None
    error: dict[str, str] | None = None


class EngineHealth(BaseModel):
    available: bool
    path: str
    version: str | None = None
    detail: str | None = None


class Health(BaseModel):
    status: Literal["ok"]
    engine: EngineHealth
