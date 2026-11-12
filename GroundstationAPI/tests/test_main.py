from fastapi.testclient import TestClient

from main import app


client = TestClient(app)


def test_dashboard_and_health_are_available() -> None:
    dashboard = client.get("/")
    health = client.get("/api/health")

    assert dashboard.status_code == 200
    assert "Mini Ground Station" in dashboard.text
    assert health.json() == {"status": "ok"}


def test_telemetry_can_be_received_and_read_back() -> None:
    sample = {
        "latitude": 34.7304,
        "longitude": -86.5861,
        "altitude_m": 25,
        "heading_deg": 90,
        "ground_speed_mps": 3.5,
        "battery_percent": 85,
        "flight_mode": "HOLD",
    }

    posted = client.post("/api/telemetry", json=sample)
    latest = client.get("/api/telemetry/latest")

    assert posted.status_code == 200
    assert posted.json()["latitude"] == sample["latitude"]
    assert posted.json()["flight_mode"] == sample["flight_mode"]
    assert posted.json()["received_at"]
    assert latest.json() == posted.json()


def test_tracker_telemetry_includes_optional_target_position() -> None:
    sample = {
        "latitude": 34.7304,
        "longitude": -86.5861,
        "altitude_m": 25,
        "heading_deg": 90,
        "target_latitude": 34.7305,
        "target_longitude": -86.586,
        "target_altitude_m": 24,
        "target_distance_m": 3.5,
    }

    response = client.post("/api/telemetry", json=sample)

    assert response.status_code == 200
    assert response.json()["target_latitude"] == sample["target_latitude"]
    assert response.json()["target_distance_m"] == sample["target_distance_m"]
    assert response.json()["ground_speed_mps"] is None


def test_invalid_telemetry_is_rejected() -> None:
    response = client.post(
        "/api/telemetry",
        json={
            "latitude": 91,
            "longitude": -86,
            "altitude_m": 25,
            "heading_deg": 90,
            "ground_speed_mps": 3.5,
            "battery_percent": 85,
            "flight_mode": "HOLD",
        },
    )

    assert response.status_code == 422
