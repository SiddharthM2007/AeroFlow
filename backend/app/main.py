"""AeroFlow HTTP API.

Run with:  uvicorn app.main:app --reload   (from the backend/ directory)
"""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

from fastapi import FastAPI, File, HTTPException, Request, UploadFile, status
from fastapi.concurrency import run_in_threadpool
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .engine import EngineError, EngineUnavailable, GridEngine
from .jobs import JobStore
from .models_store import InvalidModel, ModelStore, inspect_model
from .schemas import EngineHealth, GridJob, GridJobRequest, Health, ModelInfo
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
    jobs = JobStore(settings.max_jobs_retained)

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
        if req.model_id is not None and models.get(req.model_id) is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"model '{req.model_id}' not found")

        created = datetime.now(timezone.utc)
        result, error = None, None
        try:
            result = await run_in_threadpool(engine.generate, req.engine_config(), req.include_cells)
        except EngineUnavailable as exc:
            raise HTTPException(status.HTTP_503_SERVICE_UNAVAILABLE, str(exc)) from exc
        except EngineError as exc:
            error = {"code": exc.code, "message": exc.message}

        job = GridJob(
            job_id=uuid.uuid4().hex,
            status="failed" if error else "completed",
            created_at=created,
            finished_at=datetime.now(timezone.utc),
            request=req,
            result=result,
            error=error,
        )
        jobs.add(job)
        return job

    @app.get("/api/grid-jobs/{job_id}", response_model=GridJob)
    def get_grid_job(job_id: str) -> GridJob:
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status.HTTP_404_NOT_FOUND, f"grid job '{job_id}' not found")
        return job

    return app


app = create_app()
