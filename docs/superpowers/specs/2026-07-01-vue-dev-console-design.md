# Vue Developer Console Design

## Goal

Build a read-only developer console for the cleaner robot project, focused first on RTK path-correction debugging with a Vue-based real-time preview.

## Scope

The first version creates an independent `dev_console` module. It does not change the robot control loop, does not send movement commands, and does not include camera video. It reads runtime state, task segment data, RTK position, and correction math, then renders a browser UI for debugging.

## Architecture

- `dev_console/app.py` runs a small Flask app on a separate port, default `7900`.
- `dev_console/correction_state.py` builds a read-only correction snapshot from Redis-like state and pure helper inputs.
- `dev_console/static/index.html` hosts a Vue 3 UI using the browser build from CDN.
- The Vue page polls `/dev/correction/state` every 500 ms and draws a Canvas preview.
- Tests cover the backend snapshot math and trace handling without requiring hardware.

## Correction Preview

The main page shows:

- Current segment line from `startLat/startLon` to `endLat/endLon`.
- Current vehicle point and heading arrow.
- Target heading arrow.
- Cross-track error line from vehicle to route.
- Recent RTK trace tail.
- Numeric values for `headingError`, `cte`, `zSpeed`, `distanceToTarget`, `signedRemaining`, `globalGo`, mission, parking, and RTK fixed status.

The preview is intentionally RTK-only. Camera video and vision detection overlays are out of scope for this version.

## Data Flow

`main.py` and existing robot threads continue writing state to Redis. The dev console reads:

- `currentLocation`
- `mission`
- `parking`
- `runtimeDetail`
- `edgeDistanceToTargetM`
- current `taskList` item, inferred from `curTaskIndex`

For local debugging without the robot process, the dev console also supports loading `config.json` as a fallback task source.

## Safety

The first version is read-only. It exposes no endpoints that move the vehicle, alter Redis, send MQTT commands, or write serial frames.

## Testing

Tests should verify:

- Correction state returns empty segment data when no task exists.
- A valid task and location produce heading error, cross-track error, distance, signed remaining, and z-speed.
- Trace storage keeps recent points and does not grow unbounded.
- The Flask endpoint returns JSON without requiring real Redis hardware when passed a local test client.
