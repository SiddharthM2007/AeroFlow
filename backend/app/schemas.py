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
FieldId = Literal[
    "divergence_diagnostic",
    "pressure",
    "divergence_before",
    "divergence_after",
    "velocity_magnitude",
    "sim_pressure",
    "sim_velocity_magnitude",
    "sim_divergence",
    "sim_u",
    "sim_v",
    "sim_w",
]
PROJECTION_FIELDS = {"pressure", "divergence_before", "divergence_after", "velocity_magnitude"}
SIMULATION_FIELDS = {"sim_pressure", "sim_velocity_magnitude", "sim_divergence", "sim_u", "sim_v", "sim_w"}
FACE_FIELDS = {"sim_u", "sim_v", "sim_w"}
JobStatus = Literal["queued", "running", "completed", "stopped", "failed", "cancelled"]
FINAL_STATUSES = {"completed", "stopped", "failed", "cancelled"}

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


# Pressure Poisson solver (Milestone 7): "auto" picks MG-PCG from 1,000 fluid cells, else Jacobi-PCG.
# Both solve the same equations to the same tolerance; MG-PCG only accelerates the solve.
PressureSolver = Literal["auto", "mgpcg", "jacobi_pcg"]


class ProjectionSettings(StrictModel):
    """Pressure projection of the initialized field (Milestone 5). Mirrors the engine's checks."""

    inlet_velocity: Annotated[float, Field(ge=0, allow_inf_nan=False)] = Field(
        default=5.0, description="m/s along +x; also used by the divergence diagnostic"
    )
    density: Annotated[float, Field(gt=0, allow_inf_nan=False)] = Field(default=1.225, description="kg/m^3")
    dt: Annotated[float, Field(gt=0, allow_inf_nan=False)] | None = Field(
        default=None, description="s; default CFL 0.5 * min(dx, dy, dz) / inlet_velocity"
    )
    tolerance: Annotated[float, Field(gt=0, lt=1, allow_inf_nan=False)] = Field(
        default=1e-10, description="relative residual ||b - A phi|| / ||b||"
    )
    max_iterations: Annotated[int, Field(ge=1, le=1_000_000, strict=True)] = 10_000
    pressure_solver: PressureSolver = Field(
        default="auto", description="'auto' (MG-PCG from 1,000 fluid cells), 'mgpcg' or 'jacobi_pcg'"
    )

    @model_validator(mode="after")
    def _dt_needs_inflow(self) -> ProjectionSettings:
        if self.dt is None and self.inlet_velocity == 0:
            raise ValueError("dt: must be given explicitly when inlet_velocity is 0")
        return self


class SimulationSettings(StrictModel):
    """Time-dependent Navier-Stokes run (Milestone 6). Mirrors the engine's checks."""

    inlet_velocity: Annotated[float, Field(ge=0, allow_inf_nan=False)] = 5.0
    density: Annotated[float, Field(gt=0, allow_inf_nan=False)] = 1.225
    viscosity: Annotated[float, Field(ge=0, allow_inf_nan=False)] = Field(default=1.5e-5, description="m^2/s")
    duration: Annotated[float, Field(gt=0, allow_inf_nan=False)] = Field(default=0.5, description="s")
    frame_interval: Annotated[float, Field(gt=0, allow_inf_nan=False)] = Field(default=0.05, description="s")
    cfl: Annotated[float, Field(gt=0, le=1, allow_inf_nan=False)] = 0.5
    dt: Annotated[float, Field(gt=0, allow_inf_nan=False)] | None = Field(
        default=None, description="s; upper limit per step, rejected if unstable at t = 0"
    )
    max_steps: Annotated[int, Field(ge=1, le=1_000_000, strict=True)] = 2000
    tolerance: Annotated[float, Field(gt=0, lt=1, allow_inf_nan=False)] = 1e-8
    max_iterations: Annotated[int, Field(ge=1, le=1_000_000, strict=True)] = 10_000
    initial: Literal["uniform", "rest"] = "uniform"
    pressure_solver: PressureSolver = Field(
        default="auto", description="'auto' (MG-PCG from 1,000 fluid cells), 'mgpcg' or 'jacobi_pcg'"
    )

    @model_validator(mode="after")
    def _frame_count(self) -> SimulationSettings:
        if self.duration / self.frame_interval > 10_000:
            raise ValueError("frame_interval: duration / frame_interval must not exceed 10000 frames")
        return self

    @property
    def frame_count(self) -> int:
        import math

        return int(math.ceil(self.duration / self.frame_interval - 1e-9)) + 1


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
    fields: list[FieldId] = Field(
        default_factory=list,
        description="Numerical fields to export as AFFIELD files (opt-in). "
        "'divergence_diagnostic' is an initialized diagnostic; pressure, divergence_before, "
        "divergence_after and velocity_magnitude come from one pressure projection. None of "
        "them is a time-accurate flow solution.",
    )
    projection: ProjectionSettings | None = Field(
        default=None, description="Projection settings; engine defaults when omitted."
    )
    simulation: SimulationSettings | None = Field(
        default=None,
        description="Navier-Stokes run settings for sim_* fields (defaults when omitted). "
        "Simulation jobs run asynchronously: poll GET /api/grid-jobs/{id}.",
    )

    @model_validator(mode="after")
    def _unique_fields(self) -> GridJobRequest:
        if len(set(self.fields)) != len(self.fields):
            raise ValueError("fields: field ids must be unique")
        return self

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

    @property
    def is_simulation(self) -> bool:
        return any(f in SIMULATION_FIELDS for f in self.fields)

    @property
    def simulation_settings(self) -> SimulationSettings:
        return self.simulation or SimulationSettings()

    def engine_config(
        self, triangles_file: str | None = None, fields_dir: str | None = None
    ) -> dict[str, Any]:
        """The JSON document passed to the aeroflow-grid executable."""
        config = self.model_dump(include={"domain", "resolution"}, mode="json")
        if self.fields:
            if fields_dir is None:
                raise ValueError("field export needs an output directory")
            config["fields"] = {"output_dir": fields_dir, "requested": list(self.fields)}
            if self.projection is not None:
                config["projection"] = self.projection.model_dump(mode="json", exclude_none=True)
            if self.is_simulation:
                config["simulation"] = self.simulation_settings.model_dump(mode="json", exclude_none=True)
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
    status: JobStatus
    created_at: datetime
    finished_at: datetime | None = None
    # Simulation jobs: preflight estimate and live progress ({step, time, duration, ...}).
    estimate: dict[str, Any] | None = None
    progress: dict[str, Any] | None = None
    request: GridJobRequest
    result: dict[str, Any] | None = None
    error: dict[str, str] | None = None
    # Validated AFFIELD headers (without block offsets); data via /api/grid-jobs/{id}/fields/{name}.
    fields: list[dict[str, Any]] = Field(default_factory=list)


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
    max_field_mb: int


class Health(BaseModel):
    status: Literal["ok"]
    engine: EngineHealth
