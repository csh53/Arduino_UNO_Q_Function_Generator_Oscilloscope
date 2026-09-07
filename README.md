# Arduino UNO Q Function Generator, Oscilloscope & FFT Spectrum Analyzer

A web-based function generator, oscilloscope, and FFT spectrum analyzer
built for the Arduino UNO Q.

This project combines the STM32 MCU, Arduino Bridge, MPU-side Python backend,
and browser-based Web UI for real-time signal generation, acquisition,
measurement, FFT analysis, and visualization.

## Key Features

- Function Generator using DAC output
- Oscilloscope using ADC input
- FFT Spectrum Analyzer
- Real-time Web UI
- Arduino UNO Q MCU–MPU architecture
- ADC sampling at 2 kS/s
- DAC timer at 10 kHz
- FFT size: 16,384 samples
- Multi-browser shared control
- Network performance diagnostics

# UNO Q Function Generator & Oscilloscope

## v19.52 — Lean /api/status diagnostic output

v19.52 keeps the v19.51 acquisition, analysis, rendering, LIVE recovery, shared-control, network-diagnostic, and lean `/api/samples` behavior unchanged while hiding legacy DMA-development fields from `/api/status` output.

### v19.52 change
- `/api/status` now omits top-level `dma_*` and `adc_dma_*` development/diagnostic fields from its JSON output.
- The internal `state` and all DMA-related runtime/source code remain unchanged.
- `/api/samples` remains in the lean v19.51 form.
- No HTML behavior, MCU acquisition, Bridge transport, or MPU analysis algorithm was changed.

### Network diagnostics added to `/api/status`
A new `network` object reports a rolling 5 s view of the existing HTTP/API traffic:
- `active_clients`: estimated number of active computers, counted by unique remote IPs seen on `/api/samples` or `/api/analysis` during the last 5 s. Multiple tabs on one computer count as one computer.
- `http_rps`: JSON/API responses per second.
- `samples_rps`: `/api/samples` responses per second.
- `analysis_rps`: `/api/analysis` responses per second.
- `control_rps`: POST/control responses per second.
- `status_rps`: `/api/status` responses per second.
- `json_tx_kib_s`, `json_tx_mbps`: JSON response-body throughput. TCP/IP and HTTP-header overhead are not included.
- rolling average/max response times for all JSON/API responses, `/api/samples`, and `/api/analysis`.
- cumulative `total_http_requests` and `total_json_tx_bytes`.

The active-client estimate is intentionally based on remote IP so classroom tests can record the number of connected computers without adding browser IDs or changing the browser polling protocol.

### Diagnostic-overhead policy
- No new polling loop.
- No new worker thread.
- Existing requests only update small counters/timestamps and a bounded 5 s deque.
- Network diagnostics use a separate lock and do not change the ADC/server-ring lock path.
- Static HTML transfer is not included in the rolling JSON/API throughput because it is a one-time page load and not part of the steady-state measurement traffic.

### Preserved v19.45 behavior
- All visible controls remain shared across browsers exactly as in v19.45.
- Browser LIVE lag > 1.0 s still discards stale browser-only playback and jumps to `/api/samples?latest=1`.
- Spectrum axis begins at 0.2 Hz; spectrum is invisible <=0.3 Hz, fades from 0.3 to 0.5 Hz, and is fully visible >=0.5 Hz.
- Bridge RPC batch remains 256.
- ADC 2 kS/s, DAC 10 kHz timer, TIME 25 fps, MEASURE + COMPARE 2 Hz, FFT 2 Hz, FFT N=16384 and 250 ms analysis staggering are unchanged.
- `python/analysis_engine.py` and `sketch/sketch.ino` signal-processing/acquisition behavior are unchanged.
