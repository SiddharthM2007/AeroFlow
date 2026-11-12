"""AeroFlow HTTP API.

Run with:  uvicorn app.main:app --reload   (from the backend/ directory)
"""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, Request, UploadFile, status
from fastapi.concurrency import run_in_threadpool
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse

from .engine import EngineError, EngineUnavailable, GridEngine
from .fields import FIELD_MEDIA_TYPE, FieldFileError, FieldStore, estimate_field_bytes
from .jobs import JobStore
from .mesh_io import MeshParseError, prepare_world_mesh
from .models_store import InvalidModel, ModelStore, inspect_model
from .schemas import EngineHealth, GridJob, GridJobRequest, Health, Limits, ModelInfo
from .settings import Settings, load_settings


def _format_validation_errors(exc: RequestValidationError) -> list[dict[str, str]]:
    errors = []
    for err in exc.errors():
        field = ".".join(str(part) for part in err.get("loc", ()) if part != "body")
        message = str(err.get("msg", "invalid value")).removeprefix("Value error, ")
        if not field and ": " in message:
            # Model-level validators phrase errors as "<field>: <message>".
            field, message = message.split(": ", 1)
        errors.append({"field": field or "body", "message": message})
    return errors


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or load_settings()
    engine = GridEngine(settings.engine_path, settings.engine_timeout_s)
    models = ModelStore(settings.models_dir)
    field_store = FieldStore(settings.jobs_dir, settings.max_fields_disk_bytes)
    field_store.reset()  # jobs are in-memory; field files from a previous run are orphans
    jobs = JobStore(settings.max_jobs_retained, on_evict=field_store.remove)

    app = FastAPI(title="AeroFlow API", version="0.1.0")
    app.state.settings = settings
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(settings.cors_origins),
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.exception_handler(RequestValidationError)
    async def _validation_handler(_: Request, exc: RequestValidationError) -> JSONResponse:
        return JSONResponse(
            status_code=422,
            content={"detail": "Request validation failed", "errors": _format_validation_errors(exc)},
        )

    @app.get("/api/health", response_model=Health)
    def health() -> Health:
        info = EngineHealth(available=engine.available(), path=str(engine.path))
        if info.available:
            try:
                info.version = engine.version()
            except Exception as exc:  # noqa: BLE001 - surfaced to the client as detail
                info.available, info.detail = False, f"engine failed to report version: {exc}"
        else:
            info.detail = "engine executable not found; see README for build steps"
        return Health(status="ok", engine=info)

    @app.get("/api/limits", response_model=Limits)
    def limits() -> Limits:
        return Limits(
            max_cells=settings.max_cells,
            max_cells_per_axis=settings.max_cells_per_axis,
            max_upload_mb=settings.max_upload_bytes // (1024 * 1024),
            max_triangles=settings.max_triangles,
            max_field_mb=settings.max_field_bytes // (1024 * 1024),
        )

    @app.post("/api/models", response_model=ModelInfo, status_code=status.HTTP_201_CREATED)
    async def upload_model(file: UploadFile = File(...)) -> ModelInfo:
        filename = file.filename or ""
        if not filename:
            raise HTTPException(status.HTTP_400_BAD_REQUEST, "uploaded file has no filename")
        data = await file.read(settings.max_upload_bytes + 1)
        if len(data) > settings.max_upload_bytes:
            raise HTTPException(
                413,
                f"file exceeds the {settings.max_upload_bytes // (1024 * 1024)} MB upload limit",
            )
        try:
            info = inspect_model(filename, data)
        except InvalidModel as exc:
            raise HTTPException(status.HTTP_400_BAD_REQUEST, str(exc)) from exc
        return models.save(info, data)

    @app.get("/api/models/{model_id}", response_model=ModelInfo)
    def get_model(model_id: str) -> ModelInfo:
        info = models.get(model_id)
        if info is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"model '{model_id}' not found")
        return info

    @app.post("/api/grid-jobs", response_model=GridJob, status_code=status.HTTP_201_CREATED)
    async def create_grid_job(req: GridJobRequest) -> GridJob:
        res = req.resolution
        for axis in ("nx", "ny", "nz"):
            if getattr(res, axis) > settings.max_cells_per_axis:
                raise HTTPException(
                    422,
                    f"resolution.{axis} must be <= {settings.max_cells_per_axis}",
                )
        if res.total > settings.max_cells:
            raise HTTPException(
                422,
                f"grid has {res.total:,} cells; the server limit is {settings.max_cells:,} "
                "(set AEROFLOW_MAX_CELLS to raise it)",
            )
        info = models.get(req.model_id) if req.model_id is not None else None
        if req.model_id is not None and info is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"model '{req.model_id}' not found")
        if req.fields:
            per_field = estimate_field_bytes(res.total)
            if per_field > settings.max_field_bytes:
                raise HTTPException(
                    422,
                    f"each field would be about {per_field / 2**20:.1f} MB; the limit is "
                    f"{settings.max_field_bytes // 2**20} MB (set AEROFLOW_MAX_FIELD_MB to raise it)",
                )
            if not field_store.has_room_for(per_field * len(req.fields)):
                raise HTTPException(
                    status.HTTP_507_INSUFFICIENT_STORAGE,
                    "field storage budget is full (AEROFLOW_MAX_FIELDS_DISK_MB); older jobs' "
                    "fields are released as jobs are evicted",
                )

        created = datetime.now(timezone.utc)
        job_id = uuid.uuid4().hex
        result, error, field_headers = None, None, []
        triangles_file = None
        keep_fields = False
        try:
            if req.geometry_source == "mesh":
                triangles_file = await _prepare_mesh(req, job_id)
            fields_dir = str(field_store.job_dir(job_id).resolve()) if req.fields else None
            config = req.engine_config(str(triangles_file) if triangles_file else None, fields_dir)
            result = await run_in_threadpool(engine.generate, config, req.include_cells)
            if req.fields:
                try:
                    field_headers = field_store.register(
                        job_id, result.get("fields", []), req.resolution.model_dump(), settings.max_field_bytes
                    )
                    keep_fields = True
                except FieldFileError as exc:
                    error = {"code": "invalid_field", "message": str(exc)}
        except EngineUnavailable as exc:
            raise HTTPException(status.HTTP_503_SERVICE_UNAVAILABLE, str(exc)) from exc
        except EngineError as exc:
            error = {"code": exc.code, "message": exc.message}
        finally:
            if triangles_file is not None:
                triangles_file.unlink(missing_ok=True)
            if req.fields and not keep_fields:
                field_store.remove(job_id)  # failed or rejected: no partial field files remain

        job = GridJob(
            job_id=job_id,
            status="failed" if error else "completed",
            created_at=created,
            finished_at=datetime.now(timezone.utc),
            request=req,
            result=result,
            error=error,
            fields=field_headers,
        )
        jobs.add(job)
        return job

    async def _prepare_mesh(req: GridJobRequest, job_id: str) -> Path:
        """Writes the world-space triangle file for a mesh job; returns its path."""
        assert req.model_id is not None and req.model_transform is not None  # schema-validated
        if not engine.available():
            # Fail before the (potentially slow) mesh conversion.
            raise EngineUnavailable(f"engine executable not found at {engine.path}")
        info = models.get(req.model_id)
        raw_path = models.raw_path(req.model_id)
        if info is None or raw_path is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"model file for '{req.model_id}' is missing")
        settings.work_dir.mkdir(parents=True, exist_ok=True)
        out = settings.work_dir / f"{job_id}.aftri"  # generated name; never the client filename
        try:
            await run_in_threadpool(
                prepare_world_mesh,
                raw_path,
                info.format,
                req.model_transform.matrix,
                settings.max_triangles,
                out,
            )
        except MeshParseError as exc:
            out.unlink(missing_ok=True)
            raise HTTPException(422, f"model_id: {exc}") from exc
        return out

    @app.get("/api/grid-jobs/{job_id}/fields")
    def list_fields(job_id: str) -> dict:
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"grid job '{job_id}' not found")
        return {"job_id": job_id, "fields": job.fields}

    @app.get("/api/grid-jobs/{job_id}/fields/{name}")
    def get_field(job_id: str, name: str, frame: int = 0) -> FileResponse:
        """Binary AFFIELD v1 file (float32 values + uint8 validity); never JSON arrays."""
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"grid job '{job_id}' not found")
        exported = {(f["name"], f["frame"]["index"]) for f in job.fields}
        if (name, frame) not in exported:
            available = ", ".join(f"{n} (frame {i})" for n, i in sorted(exported)) or "none"
            raise HTTPException(
                status.HTTP_404_NOT_FOUND,
                f"field '{name}' frame {frame} was not exported for this job; available: {available}",
            )
        path = field_store.path(job_id, name, frame)
        if path is None:
            raise HTTPException(status.HTTP_410_GONE, f"field '{name}' data is no longer stored")
        return FileResponse(path, media_type=FIELD_MEDIA_TYPE, headers={"X-AeroFlow-Field-Version": "1"})

    @app.get("/api/grid-jobs/{job_id}", response_model=GridJob)
    def get_grid_job(job_id: str) -> GridJob:
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"grid job '{job_id}' not found")
        return job

    return app


app = create_app()
