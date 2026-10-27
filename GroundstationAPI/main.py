from datetime import datetime, timezone
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field


STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="Mini Ground Station API", version="0.1.0")


class Telemetry(BaseModel):
    latitude: float = Field(ge=-90, le=90)
    longitude: float = Field(ge=-180, le=180)
    altitude_m: float
    heading_deg: float = Field(ge=0, lt=360)
    ground_speed_mps: float | None = Field(default=None, ge=0)
    battery_percent: float | None = Field(default=None, ge=0, le=100)
    flight_mode: str | None = Field(default=None, min_length=1, max_length=40)
    target_latitude: float | None = Field(default=None, ge=-90, le=90)
    target_longitude: float | None = Field(default=None, ge=-180, le=180)
    target_altitude_m: float | None = None
    target_distance_m: float | None = Field(default=None, ge=0)
    received_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


latest_telemetry: Telemetry | None = None


@app.get("/", include_in_schema=False)
def dashboard() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.post("/api/telemetry", response_model=Telemetry)
def receive_telemetry(telemetry: Telemetry) -> Telemetry:
    global latest_telemetry
    latest_telemetry = telemetry
    return telemetry


@app.get("/api/telemetry/latest", response_model=Telemetry | None)
def get_latest_telemetry() -> Telemetry | None:
    return latest_telemetry
