.PHONY: all engine backend-install frontend-install install test test-engine test-backend test-frontend dev-backend dev-frontend

all: install

engine:
	cmake -S engine -B engine/build -DCMAKE_BUILD_TYPE=Release
	cmake --build engine/build -j

backend-install:
	python3 -m venv backend/.venv
	backend/.venv/bin/pip install -r backend/requirements-dev.txt

frontend-install:
	cd frontend && npm install

install: engine backend-install frontend-install

test: test-engine test-backend test-frontend

test-engine: engine
	ctest --test-dir engine/build --output-on-failure

test-backend:
	cd backend && .venv/bin/python -m pytest -q

test-frontend:
	cd frontend && npm test && npm run build

dev-backend:
	cd backend && .venv/bin/uvicorn app.main:app --reload --port 8000

dev-frontend:
	cd frontend && npm run dev
