# Personal Project #8: Flight/Drones Telemetry Analytics Platform

**Complexity:** Medium → High
**Estimated Duration:** 6–10 weeks
**Domain:** Robotics / FPV Drones / Telemetry & Analytics
**Portfolio Impact:** ★★★★★ (extremely impressive for robotics, aerospace, or data-intensive roles)

---

## Project Goal
Build a production-grade telemetry ingestion and analytics platform that receives real-time flight data from drones (or any flight controller), stores it efficiently, automatically detects flight sessions, and computes advanced post-flight metrics focused on energy efficiency, flight stability, and pilot style fingerprinting.

---

## Core Features & Functional Requirements

### 1. Telemetry Ingestion
- Async FastAPI endpoints:
  - `POST /telemetry` (bulk JSON packets)
  - `WS /telemetry/ws` (real-time streaming)
- Support at least one protocol (MSP, CRSF, or custom JSON). Bonus for multiple.
- Per-drone authentication (API-Key or JWT)
- Schema validation with Pydantic v2
- Rate-limiting & deduplication

### 2. Storage (PostgreSQL + SQLAlchemy 2.0)
| Table              | Key Fields |
|-------------------|-----------------------------|
| `drones`          | id, name, owner, api_key    |
| `telemetry_raw`   | id, drone_id, timestamp, position[x,y,z], velocity[x,y,z], attitude[roll,pitch,yaw], throttle, voltage, current, mah_drawn, rsssi, etc. |
| `flights`         | id, drone_id, start_ts, end_ts, total_mah, duration_s, bbox, computed_metrics (JSONB) |

- Automatic flight detection: new flight when throttle > 5 % for ≥ 3 s after idle

### 3. Real-Time Layer (Redis)
- Store latest telemetry packet per drone (key: `drone:{id}:live`)
- Expire after 30 s of inactivity
- Optional Redis Pub/Sub for pushing live updates to frontend

### 4. Post-Flight Analytics Engine (Pandas + NumPy)
Triggered on flight end or manually. Results stored in `flights.computed_metrics`.

#### Energy & Power
- Total energy used (mAh & Wh)
- Average / peak power (W)
- Wh per km (efficiency score)
- Voltage sag curve under load
- Throttle → power regression

#### Stability & Smoothness
- Roll/pitch/yaw rate STD & max
- Vibration score (high-freq accel if present)
- Throttle smoothness (jerk metric)
- Oscillation/phasing detection

#### Pilot Style Fingerprint
- Average throttle position
- 0→100 % throttle time (aggressiveness)
- Yaw authority vs forward speed
- Freestyle vs Racing score (heuristic)

### 5. Observability (OpenTelemetry)
- Auto-instrument FastAPI, SQLAlchemy, Redis
- Traces → Jaeger/Tempo
- Metrics → Prometheus (ingestion rate, processing latency, errors)
- Structured JSON logging

### 6. Minimum Viable API
| Method | Endpoint                    | Description |
|--------|-----------------------------|-------------|
| GET    | `/drones`                   | List registered drones |
| GET    | `/drones/{id}/flights`      | List flights |
| GET    | `/flights/{id}`             | Full flight + analytics |
| GET    | `/drones/{id}/live`         | Latest cached telemetry |
| POST   | `/telemetry`                | Ingest packet(s) |
| WS     | `/telemetry/ws`             | Streaming endpoint |

---

## Tech Stack (Required)
- **Backend:** FastAPI (Python 3.11+)
- **ORM:** SQLAlchemy 2.0+ (async)
- **Database:** PostgreSQL 15+
- **Time-series processing:** Pandas + NumPy
- **Cache & Queue:** Redis
- **Observability:** OpenTelemetry (tracing + metrics)
- **Validation:** Pydantic v2
- **Migrations:** Alembic

---

## Stretch Goals (choose 2–4)
1. Simple frontend (React/Svelte + Three.js attitude viewer + Chart.js graphs)
2. Flight comparison overlay tool
3. PDF/HTML flight report generation
4. Anomaly detection (crash, prop strike, ESC desync)
5. Replace PostgreSQL raw storage with InfluxDB + Grafana dashboards
6. Background processing with Celery + Redis Beat
7. gRPC ingestion endpoint for ultra-low latency

---

## Skills You Will Master
- High-frequency time-series ingestion & sessioning
- Advanced Pandas analytics on physical sensor data
- Real-time caching architecture
- Production observability (tracing + metrics)
- Drone-specific domain knowledge (battery physics, control theory metrics)
- Clean API design & async Python

