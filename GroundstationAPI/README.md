# Mini Ground Station API

A FastAPI dashboard that receives vehicle and tracked-target telemetry over
HTTP, keeps the latest sample in memory, and displays it on the dashboard. It
does not send flight commands.

## Run locally

```bash
cd GroundstationAPI
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
uvicorn main:app --reload

fastapi dev
```

Open <http://127.0.0.1:8000> for the dashboard or
<http://127.0.0.1:8000/docs> for interactive API documentation. Use
`python -m pip install -r requirements-dev.txt` to install the test tools, then
run `python -m pytest` from this folder.

The app binds to localhost by default and keeps data only in memory; restarting
it clears the latest sample. Do not expose this unauthenticated test service to
the public internet or connect it to a real vehicle.

The tracker posts to `http://127.0.0.1:8000/api/telemetry` by default. Override
the URL with the tracker parameter
`-p groundstation_api_url:=http://HOST:8000/api/telemetry` when the API is
reachable at a different address. If the tracker and API run in separate
containers, use an address reachable from the tracker container rather than
`127.0.0.1`.

## API

- `GET /api/health` — reports whether the service is running.
- `POST /api/telemetry` — validates and stores the latest telemetry sample.
- `GET /api/telemetry/latest` — returns the latest sample or `null` if none has
  been received.

  Vehicle latitude, longitude, altitude, and heading are sent by the tracker.
  Tracked-target latitude, longitude, altitude, and distance are included when
  available. Ground speed, battery percentage, and flight mode remain optional
  because the tracker does not currently publish those values.

Example:

```bash
curl -X POST http://127.0.0.1:8000/api/telemetry \
  -H 'Content-Type: application/json' \
  -d '{"latitude":34.7304,"longitude":-86.5861,"altitude_m":25,"heading_deg":90,"ground_speed_mps":3.5,"battery_percent":85,"flight_mode":"HOLD"}'
```
