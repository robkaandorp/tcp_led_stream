# Copilot Instructions

## Build, test, and lint commands
- This component directory has no local build/lint/test tooling configured (no Makefile, `pyproject.toml`, `tox.ini`, or test files).
- Automated suite command: not available in this directory.
- Single-test command: not applicable (no test harness is defined here).

## High-level architecture
- `tcp_led_stream.py` is the ESPHome codegen entrypoint: it defines `CONFIG_SCHEMA`, optional sensor/binary-sensor entities, and emits C++ calls in `to_code`.
- `tcp_led_stream.h` declares `TCPLedStreamComponent`, connection/state/stat fields, and compile-time sensor/binary-sensor hooks (`#ifdef USE_SENSOR`, `#ifdef USE_BINARY_SENSOR`).
- `tcp_led_stream.cpp` implements the runtime TCP server: accepts one non-blocking client, reads a 10-byte frame header + payload incrementally, validates protocol fields, applies pixels to configured addressable lights, and publishes stats.
- `README.md` is the protocol/source-of-truth doc for frame format and YAML usage; keep code/docs aligned when protocol or config options change.

## Key conventions in this repository
- External component dependency pattern: `DEPENDENCIES = ["network", "socket"]` and `AUTO_LOAD = ["light"]`; `sensor`/`binary_sensor` remain optional and must not be auto-loaded.
- Optional entities are gated in both layers:
  - Python uses conditional imports and conditionally extends schema.
  - C++ uses compile guards plus no-op setter signatures when features are not compiled in.
- Protocol contract is fixed to:
  - magic `LEDS` (bytes 0-3),
  - version `0x01` (legacy) or `0x02` (ACK pacing) in byte 4,
  - big-endian pixel count (bytes 5-8),
  - pixel-format enum (byte 9),
  - packed pixel payload.
- In protocol version `0x02`, server sends a one-byte ACK (`0x06`) after a frame using a delay derived from largest strip size and `show_time_per_led_us` plus configurable `safety_margin_ms` (default 2).
- Safety/connection behavior is strict: invalid header/version/count or mid-frame disconnect closes the client and resets receive state.
- Multi-light behavior: one incoming frame is distributed sequentially across `light_ids` in order, then each output schedules `show()`.
- Overlap accounting supports two modes (`heuristic`, `estimate`); estimate uses `largest_led_count * show_time_per_led_us / 1000 + safety_margin_ms`.
- YAML convention from `README.md`: users must explicitly declare `socket`, and also `sensor` / `binary_sensor` when enabling those optional entities.
