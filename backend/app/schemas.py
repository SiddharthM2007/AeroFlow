"""Request/response models. Field validation mirrors the engine's config checks so
clients get precise 422 errors before the engine is ever invoked."""

from __future__ import annotations

from datetime import datetime
from typing import Annotated, Any, Literal

import math

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .transform import UNIT_TO_METERS, compose_matrix

Length = Annotated[float, Field(gt=0, allow_inf_nan=False, description="Length in meters")]
Coordinate = Annotated[float, Field(allow_inf_nan=False)]
Point3 = Annotated[
    list[Coordinate], Field(min_length=3, max_length=3, description="[x, y, z] in meters")
]
CellCount = Annotated[int, Field(ge=1, strict=True)]
PositiveScale = Annotated[float, Field(gt=0, allow_inf_nan=False)]
LengthUnit = Literal["mm", "cm", "m", "in"]
GeometrySource = Literal["procedural_box", "mesh"]

# Relative tolerance when checking the client's matrix against the parameters.
MATRIX_RTOL = 1e-9


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


class ModelTransformSpec(StrictModel):
    """World placement of an uploaded model: p_world = T·R·S·U·(p_raw − pivot).

    See app/transform.py for the exact convention. `matrix` (row-major 4×4, raw file units →
    tunnel meters) must agree with the parameters; it is what the voxelizer will consume.
    """

    units: LengthUnit
    unit_scale: Annotated[float, Field(gt=0, allow_inf_nan=False)]
    pivot: Point3 = Field(description="Raw bounding-box center, file units")
    scale: Annotated[list[PositiveScale], Field(min_length=3, max_length=3)]
    rotation_deg: Point3 = Field(description="Euler XYZ, degrees; R = Rx·Ry·Rz")
    position: Point3 = Field(description="Pivot position in tunnel coordinates, meters")
    matrix: Annotated[list[Coordinate], Field(min_length=16, max_length=16)]

    @model_validator(mode="after")
    def _consistent(self) -> ModelTransformSpec:
        expected_unit = UNIT_TO_METERS[self.units]
        if not math.isclose(self.unit_scale, expected_unit, rel_tol=1e-12):
            raise ValueError(
                f"model_transform: unit_scale {self.unit_scale} does not match units "
                f"'{self.units}' ({expected_unit})"
            )
        expected = self.composed_matrix()
        magnitude = max(1.0, max(abs(v) for v in expected))
        for n, (got, want) in enumerate(zip(self.matrix, expected)):
            if abs(got - want) > MATRIX_RTOL * magnitude:
                raise ValueError(
                    f"model_transform: matrix[{n}] = {got} is inconsistent with the "
                    f"transform parameters (expected {want})"
                )
        return self

    def composed_matrix(self) -> list[float]:
        return compose_matrix(
            self.unit_scale, self.pivot, self.scale, self.rotation_deg, self.position
        )


class GridJobRequest(StrictModel):
    domain: Domain
    resolution: Resolution
    geometry_source: GeometrySource = Field(
        default="procedural_box",
        description="'procedural_box' uses test_box; 'mesh' voxelizes the uploaded model "
        "(requires model_id and model_transform). The two are never combined.",
    )
    test_box: TestBox | None = None
    model_id: str | None = Field(
        default=None,
        description="Uploaded model to associate with the job. Recorded only; "
        "geometry is not voxelized in Milestone 1.",
    )
    model_transform: ModelTransformSpec | None = Field(
        default=None,
        description="World placement of the model (requires model_id). Used to transform "
        "the model when geometry_source is 'mesh'.",
    )
    include_cells: bool = Field(
        default=True, description="Return per-cell classification (base64 uint8)."
    )

    @model_validator(mode="after")
    def _transform_needs_model(self) -> GridJobRequest:
        if self.model_transform is not None and self.model_id is None:
            raise ValueError("model_transform: requires model_id")
        return self

    @model_validator(mode="after")
    def _geometry_source(self) -> GridJobRequest:
        if self.geometry_source == "mesh":
            if self.test_box is not None:
                raise ValueError(
                    "test_box: must be null when geometry_source is 'mesh' "
                    "(the procedural box and the imported mesh are never combined)"
                )
            if self.model_id is None:
                raise ValueError("model_id: required when geometry_source is 'mesh'")
            if self.model_transform is None:
                raise ValueError("model_transform: required when geometry_source is 'mesh'")
        return self

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

    def engine_config(self, triangles_file: str | None = None) -> dict[str, Any]:
        """The JSON document passed to the aeroflow-grid executable."""
        config = self.model_dump(include={"domain", "resolution"}, mode="json")
        if self.geometry_source == "mesh":
            if triangles_file is None:
                raise ValueError("mesh jobs need a prepared triangles file")
            config["mesh"] = {"triangles_file": triangles_file}
        else:
            config["test_box"] = self.model_dump(include={"test_box"}, mode="json")["test_box"]
        return config


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


class Limits(BaseModel):
    max_cells: int
    max_cells_per_axis: int
    max_upload_mb: int
    max_triangles: int


class Health(BaseModel):
    status: Literal["ok"]
    engine: EngineHealth
