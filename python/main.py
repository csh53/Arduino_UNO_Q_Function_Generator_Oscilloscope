from arduino.app_utils import App, Bridge

from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs
import json
import mimetypes
import threading
import time
from collections import deque

from analysis_engine import MPUAnalysisEngine

PORT = 8000
BUILD_ID = "v19.52"
ASSETS = Path(__file__).resolve().parent.parent / "assets"
UI_FILE = "index.html"

MCU_FS_NOMINAL = 2000.0
ADC_SAMPLE_RATES = (2000,)
DISPLAY_FPS = 25
RPC_BATCH_SIZE = 256
RING_CAPACITY = 32768

# MPU analysis migration stage:
# Production acquisition is fixed at 2000 S/s; analysis uses the measured MCU rate.
MPU_MEASUREMENT_INTERVAL_S = 0.50
MPU_FFT_INTERVAL_S = 0.50
MPU_ANALYSIS_PHASE_OFFSET_S = 0.25
MPU_MEASUREMENT_SAMPLES = 8192
MPU_FFT_SAMPLES = 16384

# Bridge acquisition scheduling.
# MCU returns only unseen samples from a
# 2048-sample MCU history buffer. Normal polling is deliberately paced; when a
# backlog exists the worker catches up immediately.
BRIDGE_POLL_MODE = "adaptive_incremental"
BRIDGE_NORMAL_POLL_S = 0.025
BRIDGE_STOPPED_POLL_S = 1.000
BRIDGE_CONTROL_YIELD_S = 0.005
BRIDGE_BACKOFF_MAX_S = 1.000

# Bridge health policy:
# background ADC polling is allowed to fail transiently without immediately
# declaring the hardware connection dead.
BRIDGE_START_GRACE_S = 30.0
BRIDGE_INITIAL_SYNC_DELAY_S = 3.0
BRIDGE_INITIAL_RETRY_S = 1.0
BRIDGE_POLL_FAIL_THRESHOLD = 5
BRIDGE_STOPPED_FAIL_THRESHOLD = 10

# v19.50 network diagnostics only. These counters observe the existing HTTP
# traffic; they do not add a polling loop or alter acquisition/analysis timing.
NETWORK_DIAG_WINDOW_S = 5.0
NETWORK_ACTIVE_CLIENT_WINDOW_S = 5.0
network_diag_lock = threading.Lock()
network_diag_started_at = time.monotonic()
network_diag_events = deque()
network_diag_client_last_seen = {}
network_diag_total_requests = 0
network_diag_total_tx_bytes = 0

bridge_lock = threading.Lock()
data_lock = threading.Lock()
control_pending = threading.Event()

web_sequence = 0
next_adc_counter = None
next_bridge_poll_at = 0.0
last_bridge_poll_started_at = None
adc_transport_generation = 0

APP_STARTED_AT = time.monotonic()
bridge_started_at = APP_STARTED_AT
bridge_poll_failures = 0
bridge_ever_ready = False
mcu_state_synced = False
next_initial_sync_attempt_at = bridge_started_at + BRIDGE_INITIAL_SYNC_DELAY_S


class SampleRing:
    """Fixed-size sequence-addressable ring.

    read_after() uses sequence arithmetic + direct modulo indexing, so it only
    walks the samples that are actually returned. It never scans the entire
    history on each HTTP poll.
    """

    def __init__(self, capacity):
        self.capacity = int(capacity)
        self.seq = [0] * self.capacity
        self.counter = [0] * self.capacity
        self.adc = [0] * self.capacity
        self.origin = [0] * self.capacity
        self.size = 0
        self.latest_seq = 0

    def clear(self):
        self.size = 0

    def append(self, seq, counter, adc, origin):
        idx = (int(seq) - 1) % self.capacity
        self.seq[idx] = int(seq)
        self.counter[idx] = int(counter)
        self.adc[idx] = int(adc)
        self.origin[idx] = int(origin)
        self.latest_seq = int(seq)
        if self.size < self.capacity:
            self.size += 1

    def oldest_seq(self):
        if self.size == 0:
            return self.latest_seq + 1
        return self.latest_seq - self.size + 1

    def read_after(self, after, max_items=512):
        if self.size == 0:
            return [], False

        oldest = self.oldest_seq()
        requested = int(after) + 1
        missed = requested < oldest

        # v19.23 browser-live resync: if a client has fallen behind the
        # bounded server ring, skip historical catch-up and return the newest
        # available block immediately. This keeps the oscilloscope display
        # live after tab throttling/backgrounding or any other stale client.
        if missed:
            start = max(oldest, self.latest_seq - int(max_items) + 1)
        else:
            start = requested

        end = min(self.latest_seq, start + int(max_items) - 1)

        if start > end:
            return [], missed

        rows = []
        for seq in range(start, end + 1):
            idx = (seq - 1) % self.capacity

            # A stale/wrapped slot is simply skipped rather than misreported.
            if self.seq[idx] != seq:
                continue

            rows.append({
                "seq": seq,
                "counter": self.counter[idx],
                "adc": self.adc[idx],
                "phase_origin": self.origin[idx],
            })

        return rows, missed

    def read_latest(self, max_items=512):
        """Return only the newest rows without treating it as a miss.

        Used by the browser when a hidden tab becomes visible again. The
        acquisition/analysis history on the MPU is untouched; only the browser
        view jumps back to the current live edge.
        """
        if self.size == 0:
            return []

        count = min(self.size, int(max_items))
        start = self.latest_seq - count + 1
        rows = []

        for seq in range(start, self.latest_seq + 1):
            idx = (seq - 1) % self.capacity
            if self.seq[idx] != seq:
                continue
            rows.append({
                "seq": seq,
                "counter": self.counter[idx],
                "adc": self.adc[idx],
                "phase_origin": self.origin[idx],
            })

        return rows

    def snapshot_latest(self, max_items):
        if self.size == 0:
            return {
                "adc": [],
                "counter": [],
                "origin": [],
                "sequence": self.latest_seq,
            }

        count = min(self.size, int(max_items))
        start = self.latest_seq - count + 1

        adc = []
        counter = []
        origin = []

        for seq in range(start, self.latest_seq + 1):
            idx = (seq - 1) % self.capacity
            if self.seq[idx] != seq:
                continue
            adc.append(self.adc[idx])
            counter.append(self.counter[idx])
            origin.append(self.origin[idx])

        return {
            "adc": adc,
            "counter": counter,
            "origin": origin,
            "sequence": self.latest_seq,
        }


samples = SampleRing(RING_CAPACITY)

state = {
    "build_id": BUILD_ID,
    "bridge_ok": False,
    "bridge_connecting": True,
    "bridge_error": "Starting",
    "bridge_poll_failures": 0,
    "dac_running": False,
    "adc_running": False,
    "noise_enabled": False,
    "noise_level_percent": 30,
    "mcu_fs": MCU_FS_NOMINAL,
    "mcu_fs_nominal": MCU_FS_NOMINAL,
    "adc_read_last_us": 0,
    "adc_read_max_us": 0,
    "scheduler_late_max_us": 0,
    "sample_budget_us": 1000000.0 / MCU_FS_NOMINAL,
    "dac_timer_rate_hz": 0,
    "dac_timing_mode": "unknown",
    "dac_timer_diag": 0,
    "dac_irq_connect_result": -999,
    "dac_timer_reason": "not_checked",
    "dac_isr_max_us": 0,
    "dac_irq_late_max_us": 0,
    "dac_overrun_count": 0,
    "adc_timer_rate_hz": 0,
    "adc_timing_mode": "unknown",
    "adc_timer_diag": 0,
    "adc_irq_connect_result": -999,
    "adc_timer_reason": "not_checked",
    "adc_resolution_bits": 14,
    "adc_lsb_uv": 3300000.0 / 16384.0,
    "adc_conversion_timeouts": 0,
    "adc_dma_active": False,
    "adc_dma_diag": 0,
    "adc_dma_irq_connect_result": -999,
    "adc_dma_channel": 0,
    "adc_dma_request": 0,
    "adc_dma_buffer_size": 8,
    "adc_dma_transfer_count": 0,
    "adc_dma_block_count": 0,
    "adc_dma_error_count": 0,
    "adc_overrun_count": 0,
    "adc_dma_service_last_us": 0,
    "adc_dma_service_max_us": 0,
    "adc_dma_last_status": 0,
    "adc_dma_reason": "not_checked",
    "dma_test_stage": 0,
    "dma_m2m_pass": False,
    "dma_m2m_status": 0,
    "dma_m2m_dst0": 0,
    "dma_adc_single_pass": False,
    "dma_adc_single_status": 0,
    "dma_adc_single_sample": 0,
    "dma_adc_block_pass": False,
    "dma_ccr": 0,
    "dma_ctr1": 0,
    "dma_ctr2": 0,
    "dma_cbr1": 0,
    "dma_csar": 0,
    "dma_cdar": 0,
    "dma_csr": 0,
    "adc_cfgr1": 0,
    "adc_cr": 0,
    "adc_isr": 0,
    "adc_eoc_count": 0,
    "dma_block_elapsed_us": 0,
    "dma_block_tim15_irq_count": 0,
    "dma_block_adc_requests_observed": 0,
    "dma_block_cbr1_start": 0,
    "dma_block_cbr1_end": 0,
    "dma_irq_count": 0,
    "dma_half_count": 0,
    "dma_complete_count": 0,
    "dma_idle_complete_count": 0,
    "dma_adc_4sample_pass": False,
    "dma_adc_4sample_status": 0,
    "dma_adc_4sample_elapsed_us": 0,
    "dma_adc_4sample_tim15_irq_count": 0,
    "dma_adc_4sample_cbr1_end": 0,
    "dma_adc_4sample_requests": 0,
    "dma_adc_8sample_pass": False,
    "dma_adc_8sample_status": 0,
    "dma_adc_8sample_elapsed_us": 0,
    "dma_adc_8sample_tim15_irq_count": 0,
    "dma_adc_8sample_cbr1_end": 0,
    "dma_adc_8sample_requests": 0,
    "display_fps": DISPLAY_FPS,
    "phase_origin": 0,
    "latest_counter": 0,
    "dropped_samples": 0,
    "server_ring_capacity": RING_CAPACITY,
    "bridge_poll_mode": BRIDGE_POLL_MODE,
    "bridge_poll_period_ms": BRIDGE_NORMAL_POLL_S * 1000.0,
    "bridge_poll_interval_ms": 0.0,
    "bridge_poll_duration_ms": 0.0,
    "bridge_poll_late_ms": 0.0,
    "last_batch_count": 0,
    "bridge_backlog_samples": 0,
    "bridge_backoff_ms": 0.0,
    "measure_last_ms": 0.0,
    "measure_max_ms": 0.0,
    "compare_last_ms": 0.0,
    "compare_max_ms": 0.0,
    "fft_last_ms": 0.0,
    "fft_max_ms": 0.0,
    "source_synced": False,
}

# Current requested source state for MPU-side analytic reference/spectrum.
# "changed_counter" marks the newest DAC configuration/run transition so the
# transfer analysis never mixes samples from two different source settings.
source_state = {
    "running": False,
    "waveform": "dc",
    "frequency_hz": 10.0,
    "amplitude_v": 1.0,
    "offset_v": 1.65,
    "noise_enabled": False,
    "noise_level_percent": 30,
    "changed_counter": 0,
    "generation": 0,

    # Lower analysis HOLD state. This is intentionally separate from the
    # live Function Generator controls: after DAC STOP the lower graph keeps
    # the last source that was actually active, even if controls are edited.
    "analysis_available": False,
    "hold": False,
    "held_waveform": "dc",
    "held_frequency_hz": 10.0,
    "held_amplitude_v": 1.0,
    "held_offset_v": 1.65,
}

analysis_engine = MPUAnalysisEngine()

# Browser-visible controls that are not already authoritative MCU/source state.
# Existing DAC/ADC/source/noise sharing remains untouched; this small server-side
# UI state only fills the multi-browser synchronization gaps.
UI_TIME_DIVS_MS = (0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000)
shared_ui_state = {
    "generation": 0,
    "display_hold": False,
    "dac_time_div_ms": 20.0,
    "adc_time_div_ms": 20.0,
    "vertical_auto": False,
    "vertical_lo_v": 0.0,
    "vertical_hi_v": 3.3,
    "show_ch1": True,
    "show_ch2": True,
    "compare_mode": "frequency",
    "freq_axis": "log",
    "trigger_enabled": False,
    "trigger_edge": "rise",
    "trigger_level_v": 0.0,
    "trigger_config_generation": 0,
}


def shared_ui_snapshot_locked():
    return dict(shared_ui_state)


def _nearest_ui_time_div(value):
    value = float(value)
    return float(min(UI_TIME_DIVS_MS, key=lambda x: abs(float(x) - value)))


def update_shared_ui_locked(payload):
    changed = False

    def assign(key, value):
        nonlocal changed
        if shared_ui_state.get(key) != value:
            shared_ui_state[key] = value
            changed = True

    if "display_hold" in payload:
        assign("display_hold", bool(payload["display_hold"]))
    if "dac_time_div_ms" in payload:
        assign("dac_time_div_ms", _nearest_ui_time_div(payload["dac_time_div_ms"]))
    if "adc_time_div_ms" in payload:
        assign("adc_time_div_ms", _nearest_ui_time_div(payload["adc_time_div_ms"]))
    if "vertical_auto" in payload:
        assign("vertical_auto", bool(payload["vertical_auto"]))
    if "vertical_lo_v" in payload:
        assign("vertical_lo_v", max(0.0, min(3.3, float(payload["vertical_lo_v"]))))
    if "vertical_hi_v" in payload:
        assign("vertical_hi_v", max(0.0, min(3.3, float(payload["vertical_hi_v"]))))
    if "show_ch1" in payload:
        assign("show_ch1", bool(payload["show_ch1"]))
    if "show_ch2" in payload:
        assign("show_ch2", bool(payload["show_ch2"]))
    if "compare_mode" in payload:
        mode = str(payload["compare_mode"]).lower()
        if mode in ("time", "frequency"):
            assign("compare_mode", mode)
    if "freq_axis" in payload:
        axis = str(payload["freq_axis"]).lower()
        if axis in ("log", "linear"):
            assign("freq_axis", axis)
    if "trigger_enabled" in payload:
        assign("trigger_enabled", bool(payload["trigger_enabled"]))
    if "trigger_edge" in payload:
        edge = "fall" if str(payload["trigger_edge"]).lower() == "fall" else "rise"
        assign("trigger_edge", edge)
    if "trigger_level_v" in payload:
        level = max(0.0, min(3.3, float(payload["trigger_level_v"])))
        assign("trigger_level_v", round(level, 6))
    if "trigger_config_generation" in payload:
        assign(
            "trigger_config_generation",
            max(0, int(payload["trigger_config_generation"])),
        )

    # Keep a valid vertical range even if a malformed partial request arrives.
    if shared_ui_state["vertical_hi_v"] <= shared_ui_state["vertical_lo_v"]:
        shared_ui_state["vertical_lo_v"] = 0.0
        shared_ui_state["vertical_hi_v"] = 3.3
        changed = True

    if changed:
        shared_ui_state["generation"] = int(shared_ui_state["generation"]) + 1

    return shared_ui_snapshot_locked()


def set_bridge_state(ok, error="", connecting=False):
    """Immediate state setter used by explicit control/config commands."""
    global bridge_poll_failures, bridge_ever_ready

    with data_lock:
        state["bridge_ok"] = bool(ok)
        state["bridge_connecting"] = bool(connecting)
        state["bridge_error"] = str(error or "")

        if ok:
            bridge_poll_failures = 0
            bridge_ever_ready = True
            state["bridge_poll_failures"] = 0


def mark_bridge_poll_success():
    """A single successful background poll fully restores READY state."""
    global bridge_poll_failures, bridge_ever_ready

    bridge_poll_failures = 0
    bridge_ever_ready = True

    with data_lock:
        state["bridge_ok"] = True
        state["bridge_connecting"] = False
        state["bridge_error"] = ""
        state["bridge_poll_failures"] = 0


def mark_bridge_poll_failure(error):
    """Debounce background ADC-poll failures.

    - During startup grace: CONNECTING, never BRIDGE ERROR.
    - After a prior successful connection: keep READY through a few misses.
    - When both outputs are stopped: allow a larger failure threshold.
    - Only sustained consecutive failures become a real BRIDGE ERROR.
    """
    global bridge_poll_failures

    bridge_poll_failures += 1
    now = time.monotonic()

    with data_lock:
        both_stopped = not state["dac_running"] and not state["adc_running"]
        threshold = (
            BRIDGE_STOPPED_FAIL_THRESHOLD
            if both_stopped
            else BRIDGE_POLL_FAIL_THRESHOLD
        )

        in_start_grace = (now - bridge_started_at) < BRIDGE_START_GRACE_S
        state["bridge_poll_failures"] = bridge_poll_failures

        if in_start_grace or (not bridge_ever_ready and bridge_poll_failures < threshold):
            state["bridge_ok"] = False
            state["bridge_connecting"] = True
            state["bridge_error"] = str(error or "Waiting for Bridge")
            return

        if bridge_ever_ready and bridge_poll_failures < threshold:
            # Preserve the last known-good READY/ACTIVE state during a brief
            # polling hiccup. Do not flash BRIDGE ERROR in the UI.
            state["bridge_ok"] = True
            state["bridge_connecting"] = False
            state["bridge_error"] = ""
            return

        state["bridge_ok"] = False
        state["bridge_connecting"] = False
        state["bridge_error"] = str(error or "Bridge polling failed")


def call_bridge_control(method, *args):
    """Control/config RPCs get priority over background ADC acquisition."""
    control_pending.set()
    try:
        with bridge_lock:
            return Bridge.call(method, *args)
    finally:
        control_pending.clear()


def call_bridge_poll(method, *args):
    """Background acquisition RPC.

    Returns None instead of starting a new poll while a user control command is
    waiting. This prevents the acquisition loop from repeatedly winning the
    Bridge lock between slider/RUN/STOP actions.
    """
    if control_pending.is_set():
        return None

    with bridge_lock:
        if control_pending.is_set():
            return None
        return Bridge.call(method, *args)


def clear_samples(next_counter=None):
    global next_adc_counter

    with data_lock:
        samples.clear()
        next_adc_counter = next_counter



def source_snapshot_locked():
    return dict(source_state)


def latch_source_for_analysis_locked():
    """Capture the currently configured DAC source for lower-graph HOLD."""
    source_state["analysis_available"] = True
    source_state["held_waveform"] = source_state["waveform"]
    source_state["held_frequency_hz"] = source_state["frequency_hz"]
    source_state["held_amplitude_v"] = source_state["amplitude_v"]
    source_state["held_offset_v"] = source_state["offset_v"]
    source_state["hold"] = not bool(source_state["running"])


def dac_timer_reason(active, diag, irq_result):
    if active:
        return "tim6_dynamic_irq_active"

    diag = int(diag or 0)
    irq_result = int(irq_result)

    if not (diag & (1 << 0)):
        return "not_uno_q"
    if not (diag & (1 << 1)):
        return "zephyr_irq_header_unavailable"
    if not (diag & (1 << 2)):
        return "dynamic_interrupts_disabled"
    if not (diag & (1 << 3)):
        return "tim6_clock_enable_failed"
    if not (diag & (1 << 4)):
        return "tim6_counter_not_running"
    if not (diag & (1 << 5)):
        return f"dynamic_irq_connect_failed:{irq_result}"
    if not (diag & (1 << 6)):
        return "irq_enable_failed"
    if not (diag & (1 << 7)):
        return "tim6_irq_not_firing"
    return "hardware_selftest_failed"


def adc_timer_reason(active, diag, irq_result):
    if active:
        return "tim15_trgo_adc14_active"

    diag = int(diag or 0)
    irq_result = int(irq_result)

    if diag == 0:
        return "source_preserved_runtime_disabled"
    if not (diag & (1 << 0)):
        return "not_uno_q"
    if not (diag & (1 << 1)):
        return "zephyr_irq_header_unavailable"
    if not (diag & (1 << 2)):
        return "dynamic_interrupts_disabled"
    if not (diag & (1 << 3)):
        return "tim15_clock_enable_failed"
    if not (diag & (1 << 4)):
        return "tim15_counter_not_running"
    if not (diag & (1 << 5)):
        return f"tim15_dynamic_irq_connect_failed:{irq_result}"
    if not (diag & (1 << 6)):
        return "tim15_irq_enable_failed"
    if not (diag & (1 << 7)):
        return "tim15_trgo_adc_conversion_failed"
    return "adc_hardware_selftest_failed"


def adc_dma_reason(active, diag, irq_result):
    if active:
        return "gpdma1_ch0_adc1_16bit_8sample_rearm_active"

    diag = int(diag or 0)
    irq_result = int(irq_result)

    if diag == 0:
        return "source_preserved_runtime_disabled"
    if not (diag & (1 << 0)):
        return "not_uno_q"
    if not (diag & (1 << 1)):
        return "gpdma1_clock_enable_failed"
    if not (diag & (1 << 2)):
        return "gpdma1_ch0_busy_irq_fallback"
    if not (diag & (1 << 3)):
        return "gpdma1_ram_to_ram_selftest_failed_irq_fallback"
    if not (diag & (1 << 4)):
        return f"gpdma1_ch0_irq_connect_or_enable_failed:{irq_result}"
    if not (diag & (1 << 5)):
        return "adc1_single_dma_handshake_failed_irq_fallback"
    if not (diag & (1 << 6)):
        return "adc1_dma_4_8sample_or_rearm_failed_irq_fallback"
    if not (diag & (1 << 7)):
        return "gpdma1_ch0_runtime_not_ready"
    return "gpdma1_ch0_validated_standby_irq_default"



def parse_signal_state(raw):
    parts = str(raw).strip().split(",")
    if len(parts) not in (8, 9, 12, 14, 16, 22, 36, 55, 75, 76, 79, 81):
        raise ValueError(f"Bad signal state: {raw!r}")

    waveform = max(0, min(3, int(parts[0])))
    frequency_hz = max(1.0, min(100.0, int(parts[1]) / 1000.0))
    amplitude_v = max(0.0, min(1.65, int(parts[2]) / 1000.0))
    offset_v = max(0.0, min(3.3, int(parts[3]) / 1000.0))
    dac_running = bool(int(parts[4]))
    adc_running = bool(int(parts[5]))
    phase_origin = int(parts[6])
    sample_counter = int(parts[7])

    sample_rate_hz = MCU_FS_NOMINAL
    if len(parts) >= 9:
        candidate = int(parts[8]) / 1000.0
        if 10.0 <= candidate <= 5000.0:
            sample_rate_hz = candidate

    adc_read_last_us = int(parts[9]) if len(parts) >= 12 else 0
    adc_read_max_us = int(parts[10]) if len(parts) >= 12 else 0
    scheduler_late_max_us = int(parts[11]) if len(parts) >= 12 else 0
    dac_timer_rate_hz = int(parts[12]) if len(parts) >= 14 else 0
    dac_hw_timer = bool(int(parts[13])) if len(parts) >= 14 else False
    dac_timer_diag = int(parts[14]) if len(parts) >= 16 else 0
    dac_irq_connect_result = int(parts[15]) if len(parts) >= 16 else -999

    adc_timer_rate_hz = int(parts[16]) if len(parts) >= 22 else 0
    adc_hw_timer = bool(int(parts[17])) if len(parts) >= 22 else False
    adc_timer_diag = int(parts[18]) if len(parts) >= 22 else 0
    adc_irq_connect_result = int(parts[19]) if len(parts) >= 22 else -999
    adc_resolution_bits = int(parts[20]) if len(parts) >= 22 else 14
    adc_conversion_timeouts = int(parts[21]) if len(parts) >= 22 else 0

    adc_dma_active = bool(int(parts[22])) if len(parts) >= 36 else False
    adc_dma_diag = int(parts[23]) if len(parts) >= 36 else 0
    adc_dma_irq_connect_result = int(parts[24]) if len(parts) >= 36 else -999
    adc_dma_channel = int(parts[25]) if len(parts) >= 36 else 0
    adc_dma_request = int(parts[26]) if len(parts) >= 36 else 0
    adc_dma_buffer_size = int(parts[27]) if len(parts) >= 36 else 8
    adc_dma_transfer_count = int(parts[28]) if len(parts) >= 36 else 0
    adc_dma_block_count = int(parts[29]) if len(parts) >= 36 else 0
    adc_dma_error_count = int(parts[30]) if len(parts) >= 36 else 0
    adc_overrun_count = int(parts[31]) if len(parts) >= 36 else 0
    adc_dma_service_last_us = int(parts[32]) if len(parts) >= 36 else 0
    adc_dma_service_max_us = int(parts[33]) if len(parts) >= 36 else 0
    adc_dma_last_status = int(parts[34]) if len(parts) >= 36 else 0
    adc_latest_counter = int(parts[35]) if len(parts) >= 36 else sample_counter

    dma_test_stage = int(parts[36]) if len(parts) >= 55 else 0
    dma_m2m_pass = bool(int(parts[37])) if len(parts) >= 55 else False
    dma_m2m_status = int(parts[38]) if len(parts) >= 55 else 0
    dma_m2m_dst0 = int(parts[39]) if len(parts) >= 55 else 0
    dma_adc_single_pass = bool(int(parts[40])) if len(parts) >= 55 else False
    dma_adc_single_status = int(parts[41]) if len(parts) >= 55 else 0
    dma_adc_single_sample = int(parts[42]) if len(parts) >= 55 else 0
    dma_adc_block_pass = bool(int(parts[43])) if len(parts) >= 55 else False
    dma_ccr = int(parts[44]) if len(parts) >= 55 else 0
    dma_ctr1 = int(parts[45]) if len(parts) >= 55 else 0
    dma_ctr2 = int(parts[46]) if len(parts) >= 55 else 0
    dma_cbr1 = int(parts[47]) if len(parts) >= 55 else 0
    dma_csar = int(parts[48]) if len(parts) >= 55 else 0
    dma_cdar = int(parts[49]) if len(parts) >= 55 else 0
    dma_csr = int(parts[50]) if len(parts) >= 55 else 0
    adc_cfgr1 = int(parts[51]) if len(parts) >= 55 else 0
    adc_cr = int(parts[52]) if len(parts) >= 55 else 0
    adc_isr = int(parts[53]) if len(parts) >= 55 else 0
    adc_eoc_count = int(parts[54]) if len(parts) >= 55 else 0

    dma_block_elapsed_us = int(parts[55]) if len(parts) >= 75 else 0
    dma_block_tim15_irq_count = int(parts[56]) if len(parts) >= 75 else 0
    dma_block_adc_requests_observed = int(parts[57]) if len(parts) >= 75 else 0
    dma_block_cbr1_start = int(parts[58]) if len(parts) >= 75 else 0
    dma_block_cbr1_end = int(parts[59]) if len(parts) >= 75 else 0
    dma_irq_count = int(parts[60]) if len(parts) >= 75 else 0
    dma_half_count = int(parts[61]) if len(parts) >= 75 else 0
    dma_complete_count = int(parts[62]) if len(parts) >= 75 else 0
    dma_adc_4sample_pass = bool(int(parts[63])) if len(parts) >= 75 else False
    dma_adc_4sample_status = int(parts[64]) if len(parts) >= 75 else 0
    dma_adc_4sample_elapsed_us = int(parts[65]) if len(parts) >= 75 else 0
    dma_adc_4sample_tim15_irq_count = int(parts[66]) if len(parts) >= 75 else 0
    dma_adc_4sample_cbr1_end = int(parts[67]) if len(parts) >= 75 else 0
    dma_adc_4sample_requests = int(parts[68]) if len(parts) >= 75 else 0
    dma_adc_8sample_pass = bool(int(parts[69])) if len(parts) >= 75 else False
    dma_adc_8sample_status = int(parts[70]) if len(parts) >= 75 else 0
    dma_adc_8sample_elapsed_us = int(parts[71]) if len(parts) >= 75 else 0
    dma_adc_8sample_tim15_irq_count = int(parts[72]) if len(parts) >= 75 else 0
    dma_adc_8sample_cbr1_end = int(parts[73]) if len(parts) >= 75 else 0
    dma_adc_8sample_requests = int(parts[74]) if len(parts) >= 75 else 0
    dma_idle_complete_count = int(parts[75]) if len(parts) >= 76 else 0
    dac_isr_max_us = int(parts[76]) if len(parts) >= 79 else 0
    dac_irq_late_max_us = int(parts[77]) if len(parts) >= 79 else 0
    dac_overrun_count = int(parts[78]) if len(parts) >= 79 else 0
    noise_enabled = bool(int(parts[79])) if len(parts) >= 81 else False
    noise_level_percent = max(0, min(100, int(parts[80]))) if len(parts) >= 81 else 30

    # A fallback must never advertise a fake hardware timer rate.
    if not dac_hw_timer:
        dac_timer_rate_hz = 0
    if not adc_hw_timer:
        adc_timer_rate_hz = 0

    waveform_name = {
        0: "dc",
        1: "sine",
        2: "square",
        3: "triangle",
    }.get(waveform, "dc")

    return {
        "waveform": waveform_name,
        "frequency_hz": frequency_hz,
        "amplitude_v": amplitude_v,
        "offset_v": offset_v,
        "dac_running": dac_running,
        "adc_running": adc_running,
        "phase_origin": phase_origin,
        "sample_counter": sample_counter,
        "sample_rate_hz": sample_rate_hz,
        "adc_read_last_us": adc_read_last_us,
        "adc_read_max_us": adc_read_max_us,
        "scheduler_late_max_us": scheduler_late_max_us,
        "dac_timer_rate_hz": dac_timer_rate_hz,
        "dac_hw_timer": dac_hw_timer,
        "dac_timer_diag": dac_timer_diag,
        "dac_irq_connect_result": dac_irq_connect_result,
        "dac_timer_reason": dac_timer_reason(
            dac_hw_timer,
            dac_timer_diag,
            dac_irq_connect_result,
        ),
        "dac_isr_max_us": dac_isr_max_us,
        "dac_irq_late_max_us": dac_irq_late_max_us,
        "dac_overrun_count": dac_overrun_count,
        "noise_enabled": noise_enabled,
        "noise_level_percent": noise_level_percent,
        "adc_timer_rate_hz": adc_timer_rate_hz,
        "adc_hw_timer": adc_hw_timer,
        "adc_timer_diag": adc_timer_diag,
        "adc_irq_connect_result": adc_irq_connect_result,
        "adc_timer_reason": adc_timer_reason(
            adc_hw_timer,
            adc_timer_diag,
            adc_irq_connect_result,
        ),
        "adc_resolution_bits": adc_resolution_bits,
        "adc_conversion_timeouts": adc_conversion_timeouts,
        "adc_dma_active": adc_dma_active,
        "adc_dma_diag": adc_dma_diag,
        "adc_dma_irq_connect_result": adc_dma_irq_connect_result,
        "adc_dma_channel": adc_dma_channel,
        "adc_dma_request": adc_dma_request,
        "adc_dma_buffer_size": adc_dma_buffer_size,
        "adc_dma_transfer_count": adc_dma_transfer_count,
        "adc_dma_block_count": adc_dma_block_count,
        "adc_dma_error_count": adc_dma_error_count,
        "adc_overrun_count": adc_overrun_count,
        "adc_dma_service_last_us": adc_dma_service_last_us,
        "adc_dma_service_max_us": adc_dma_service_max_us,
        "adc_dma_last_status": adc_dma_last_status,
        "adc_dma_reason": adc_dma_reason(
            adc_dma_active,
            adc_dma_diag,
            adc_dma_irq_connect_result,
        ),
        "adc_latest_counter": adc_latest_counter,
        "dma_test_stage": dma_test_stage,
        "dma_m2m_pass": dma_m2m_pass,
        "dma_m2m_status": dma_m2m_status,
        "dma_m2m_dst0": dma_m2m_dst0,
        "dma_adc_single_pass": dma_adc_single_pass,
        "dma_adc_single_status": dma_adc_single_status,
        "dma_adc_single_sample": dma_adc_single_sample,
        "dma_adc_block_pass": dma_adc_block_pass,
        "dma_ccr": dma_ccr,
        "dma_ctr1": dma_ctr1,
        "dma_ctr2": dma_ctr2,
        "dma_cbr1": dma_cbr1,
        "dma_csar": dma_csar,
        "dma_cdar": dma_cdar,
        "dma_csr": dma_csr,
        "adc_cfgr1": adc_cfgr1,
        "adc_cr": adc_cr,
        "adc_isr": adc_isr,
        "adc_eoc_count": adc_eoc_count,
        "dma_block_elapsed_us": dma_block_elapsed_us,
        "dma_block_tim15_irq_count": dma_block_tim15_irq_count,
        "dma_block_adc_requests_observed": dma_block_adc_requests_observed,
        "dma_block_cbr1_start": dma_block_cbr1_start,
        "dma_block_cbr1_end": dma_block_cbr1_end,
        "dma_irq_count": dma_irq_count,
        "dma_half_count": dma_half_count,
        "dma_complete_count": dma_complete_count,
        "dma_idle_complete_count": dma_idle_complete_count,
        "dma_adc_4sample_pass": dma_adc_4sample_pass,
        "dma_adc_4sample_status": dma_adc_4sample_status,
        "dma_adc_4sample_elapsed_us": dma_adc_4sample_elapsed_us,
        "dma_adc_4sample_tim15_irq_count": dma_adc_4sample_tim15_irq_count,
        "dma_adc_4sample_cbr1_end": dma_adc_4sample_cbr1_end,
        "dma_adc_4sample_requests": dma_adc_4sample_requests,
        "dma_adc_8sample_pass": dma_adc_8sample_pass,
        "dma_adc_8sample_status": dma_adc_8sample_status,
        "dma_adc_8sample_elapsed_us": dma_adc_8sample_elapsed_us,
        "dma_adc_8sample_tim15_irq_count": dma_adc_8sample_tim15_irq_count,
        "dma_adc_8sample_cbr1_end": dma_adc_8sample_cbr1_end,
        "dma_adc_8sample_requests": dma_adc_8sample_requests,
    }


def sync_source_from_mcu():
    global next_adc_counter, adc_transport_generation, mcu_state_synced

    actual = parse_signal_state(
        call_bridge_control("get_signal_state")
    )

    with data_lock:
        source_state["running"] = actual["dac_running"]
        source_state["waveform"] = actual["waveform"]
        source_state["frequency_hz"] = actual["frequency_hz"]
        source_state["amplitude_v"] = actual["amplitude_v"]
        source_state["offset_v"] = actual["offset_v"]
        source_state["noise_enabled"] = bool(actual["noise_enabled"])
        source_state["noise_level_percent"] = int(actual["noise_level_percent"])
        source_state["changed_counter"] = actual["phase_origin"]
        source_state["generation"] += 1

        if actual["dac_running"]:
            latch_source_for_analysis_locked()
            source_state["hold"] = False
        else:
            # After a Linux/server restart there is no trustworthy knowledge
            # of a pre-restart DAC capture, so start without a synthetic hold.
            source_state["analysis_available"] = False
            source_state["hold"] = False

        state["dac_running"] = actual["dac_running"]
        state["adc_running"] = actual["adc_running"]
        state["noise_enabled"] = bool(actual["noise_enabled"])
        state["noise_level_percent"] = int(actual["noise_level_percent"])
        state["phase_origin"] = actual["phase_origin"]
        state["latest_counter"] = actual["adc_latest_counter"]
        state["mcu_fs"] = float(actual["sample_rate_hz"])
        state["mcu_fs_nominal"] = float(actual["sample_rate_hz"])
        state["sample_budget_us"] = 1000000.0 / max(1.0, state["mcu_fs"])
        state["adc_read_last_us"] = int(actual["adc_read_last_us"])
        state["adc_read_max_us"] = int(actual["adc_read_max_us"])
        state["scheduler_late_max_us"] = int(actual["scheduler_late_max_us"])
        state["dac_timer_rate_hz"] = int(actual["dac_timer_rate_hz"])
        state["dac_timing_mode"] = (
            "hw_timer"
            if actual["dac_hw_timer"]
            else "software_fallback"
        )
        state["dac_timer_diag"] = int(actual["dac_timer_diag"])
        state["dac_irq_connect_result"] = int(
            actual["dac_irq_connect_result"]
        )
        state["dac_timer_reason"] = str(actual["dac_timer_reason"])
        state["dac_isr_max_us"] = int(actual["dac_isr_max_us"])
        state["dac_irq_late_max_us"] = int(actual["dac_irq_late_max_us"])
        state["dac_overrun_count"] = int(actual["dac_overrun_count"])

        state["adc_timer_rate_hz"] = int(actual["adc_timer_rate_hz"])
        if actual["adc_dma_active"]:
            state["adc_timing_mode"] = "hw_trigger_dma_block_ring"
        elif actual["adc_hw_timer"]:
            state["adc_timing_mode"] = "hw_trigger_irq_read"
        else:
            state["adc_timing_mode"] = "software_fallback"
        state["adc_timer_diag"] = int(actual["adc_timer_diag"])
        state["adc_irq_connect_result"] = int(
            actual["adc_irq_connect_result"]
        )
        state["adc_timer_reason"] = str(actual["adc_timer_reason"])
        state["adc_resolution_bits"] = int(actual["adc_resolution_bits"])
        state["adc_lsb_uv"] = (
            3300000.0 / float(1 << state["adc_resolution_bits"])
        )
        state["adc_conversion_timeouts"] = int(
            actual["adc_conversion_timeouts"]
        )
        state["adc_dma_active"] = bool(actual["adc_dma_active"])
        state["adc_dma_diag"] = int(actual["adc_dma_diag"])
        state["adc_dma_irq_connect_result"] = int(
            actual["adc_dma_irq_connect_result"]
        )
        state["adc_dma_channel"] = int(actual["adc_dma_channel"])
        state["adc_dma_request"] = int(actual["adc_dma_request"])
        state["adc_dma_buffer_size"] = int(actual["adc_dma_buffer_size"])
        state["adc_dma_transfer_count"] = int(
            actual["adc_dma_transfer_count"]
        )
        state["adc_dma_block_count"] = int(actual["adc_dma_block_count"])
        state["adc_dma_error_count"] = int(actual["adc_dma_error_count"])
        state["adc_overrun_count"] = int(actual["adc_overrun_count"])
        state["adc_dma_service_last_us"] = int(
            actual["adc_dma_service_last_us"]
        )
        state["adc_dma_service_max_us"] = int(
            actual["adc_dma_service_max_us"]
        )
        state["adc_dma_last_status"] = int(actual["adc_dma_last_status"])
        state["adc_dma_reason"] = str(actual["adc_dma_reason"])
        state["dma_test_stage"] = int(actual["dma_test_stage"])
        state["dma_m2m_pass"] = bool(actual["dma_m2m_pass"])
        state["dma_m2m_status"] = int(actual["dma_m2m_status"])
        state["dma_m2m_dst0"] = int(actual["dma_m2m_dst0"])
        state["dma_adc_single_pass"] = bool(actual["dma_adc_single_pass"])
        state["dma_adc_single_status"] = int(actual["dma_adc_single_status"])
        state["dma_adc_single_sample"] = int(actual["dma_adc_single_sample"])
        state["dma_adc_block_pass"] = bool(actual["dma_adc_block_pass"])
        state["dma_ccr"] = int(actual["dma_ccr"])
        state["dma_ctr1"] = int(actual["dma_ctr1"])
        state["dma_ctr2"] = int(actual["dma_ctr2"])
        state["dma_cbr1"] = int(actual["dma_cbr1"])
        state["dma_csar"] = int(actual["dma_csar"])
        state["dma_cdar"] = int(actual["dma_cdar"])
        state["dma_csr"] = int(actual["dma_csr"])
        state["adc_cfgr1"] = int(actual["adc_cfgr1"])
        state["adc_cr"] = int(actual["adc_cr"])
        state["adc_isr"] = int(actual["adc_isr"])
        state["adc_eoc_count"] = int(actual["adc_eoc_count"])
        state["dma_block_elapsed_us"] = int(actual["dma_block_elapsed_us"])
        state["dma_block_tim15_irq_count"] = int(actual["dma_block_tim15_irq_count"])
        state["dma_block_adc_requests_observed"] = int(actual["dma_block_adc_requests_observed"])
        state["dma_block_cbr1_start"] = int(actual["dma_block_cbr1_start"])
        state["dma_block_cbr1_end"] = int(actual["dma_block_cbr1_end"])
        state["dma_irq_count"] = int(actual["dma_irq_count"])
        state["dma_half_count"] = int(actual["dma_half_count"])
        state["dma_complete_count"] = int(actual["dma_complete_count"])
        state["dma_idle_complete_count"] = int(actual["dma_idle_complete_count"])
        state["dma_adc_4sample_pass"] = bool(actual["dma_adc_4sample_pass"])
        state["dma_adc_4sample_status"] = int(actual["dma_adc_4sample_status"])
        state["dma_adc_4sample_elapsed_us"] = int(actual["dma_adc_4sample_elapsed_us"])
        state["dma_adc_4sample_tim15_irq_count"] = int(actual["dma_adc_4sample_tim15_irq_count"])
        state["dma_adc_4sample_cbr1_end"] = int(actual["dma_adc_4sample_cbr1_end"])
        state["dma_adc_4sample_requests"] = int(actual["dma_adc_4sample_requests"])
        state["dma_adc_8sample_pass"] = bool(actual["dma_adc_8sample_pass"])
        state["dma_adc_8sample_status"] = int(actual["dma_adc_8sample_status"])
        state["dma_adc_8sample_elapsed_us"] = int(actual["dma_adc_8sample_elapsed_us"])
        state["dma_adc_8sample_tim15_irq_count"] = int(actual["dma_adc_8sample_tim15_irq_count"])
        state["dma_adc_8sample_cbr1_end"] = int(actual["dma_adc_8sample_cbr1_end"])
        state["dma_adc_8sample_requests"] = int(actual["dma_adc_8sample_requests"])
        state["source_synced"] = True

        # Linux may restart while the MCU keeps running. Resume acquisition
        # from "now", never by replaying stale MCU history.
        samples.clear()
        adc_transport_generation += 1
        next_adc_counter = (
            actual["adc_latest_counter"]
            if actual["adc_running"]
            else None
        )

        mcu_state_synced = True

    set_bridge_state(True)
    return actual


def read_dac_diag():
    """Read DAC DDS/output diagnostics on demand without enlarging the hot sample RPC."""
    raw = str(call_bridge_control("get_dac_diag"))
    parts = raw.strip().split(",")
    if len(parts) != 15:
        raise ValueError(f"Bad DAC diag: {raw!r}")

    keys = (
        "frequency_mHz",
        "timer_rate_hz",
        "phase_step",
        "dds_actual_mHz",
        "samples_per_cycle_x1000",
        "offset_code",
        "amplitude_code",
        "observed_min_code",
        "observed_max_code",
        "clip_count",
        "irq_count",
        "timer_tick_count",
        "isr_max_us",
        "irq_late_max_us",
        "overrun_count",
    )
    values = [int(x) for x in parts]
    result = dict(zip(keys, values))
    result["frequency_hz"] = result["frequency_mHz"] / 1000.0
    result["dds_actual_hz"] = result["dds_actual_mHz"] / 1000.0
    result["samples_per_cycle"] = result["samples_per_cycle_x1000"] / 1000.0
    result["lut_size"] = 256
    result["dac_bits"] = 12
    return result


def apply_config(payload):
    waveform = max(0, min(3, int(payload.get("waveform", 0))))
    frequency_hz = float(round(max(1.0, min(100.0, float(payload.get("frequency_hz", 10.0))))))
    amplitude_v = max(0.0, min(1.65, float(payload.get("amplitude_v", 1.0))))
    offset_v = max(0.0, min(3.3, float(payload.get("offset_v", 1.65))))

    phase_origin = int(call_bridge_control(
        "set_signal",
        waveform,
        int(round(frequency_hz * 1000.0)),
        int(round(amplitude_v * 1000.0)),
        int(round(offset_v * 1000.0)),
    ))

    waveform_name = {
        0: "dc",
        1: "sine",
        2: "square",
        3: "triangle",
    }.get(waveform, "dc")

    with data_lock:
        state["phase_origin"] = phase_origin

        source_state["waveform"] = waveform_name
        source_state["frequency_hz"] = frequency_hz
        source_state["amplitude_v"] = amplitude_v
        source_state["offset_v"] = offset_v
        source_state["changed_counter"] = phase_origin
        source_state["generation"] += 1

        if source_state["running"]:
            latch_source_for_analysis_locked()
            source_state["hold"] = False

        current_seq = web_sequence

    # Source controls are DAC-side actions.
    # Keep ADC history intact, including when the oscilloscope is STOPPED.
    set_bridge_state(True)

    with data_lock:
        applied_source = source_snapshot_locked()

    return {
        "ok": True,
        "phase_origin": phase_origin,
        "sequence": current_seq,
        "source": applied_source,
    }


def set_noise(enabled, level_percent):
    enabled = bool(enabled)
    level_percent = int(round(max(0.0, min(100.0, float(level_percent)))))

    raw = str(call_bridge_control(
        "set_noise",
        1 if enabled else 0,
        level_percent,
    )).strip()

    actual_enabled = enabled
    actual_level = level_percent
    parts = raw.split(",") if raw else []
    if len(parts) >= 2:
        actual_enabled = bool(int(parts[0]))
        actual_level = max(0, min(100, int(parts[1])))

    with data_lock:
        state["noise_enabled"] = actual_enabled
        state["noise_level_percent"] = actual_level

        source_state["noise_enabled"] = actual_enabled
        source_state["noise_level_percent"] = actual_level
        # Noise is a source-state change, but it intentionally does not change
        # changed_counter/phase_origin because DDS phase must remain continuous.
        source_state["generation"] += 1

        applied_source = source_snapshot_locked()

    set_bridge_state(True)

    return {
        "ok": True,
        "noise_enabled": actual_enabled,
        "noise_level_percent": actual_level,
        "source": applied_source,
    }


def set_dac_run(running):
    phase_origin = int(call_bridge_control("set_dac_run", 1 if running else 0))

    with data_lock:
        state["dac_running"] = bool(running)
        state["phase_origin"] = phase_origin

        source_state["running"] = bool(running)
        source_state["changed_counter"] = phase_origin
        source_state["generation"] += 1

        if running:
            latch_source_for_analysis_locked()
            source_state["hold"] = False
        else:
            # Do not replace held_* with later STOP-state edits. The lower
            # graph represents the last source that was physically active.
            source_state["hold"] = bool(source_state["analysis_available"])

        current_seq = web_sequence

    # IMPORTANT: DAC control never clears ADC history.
    set_bridge_state(True)

    with data_lock:
        applied_source = source_snapshot_locked()

    return {
        "ok": True,
        "running": bool(running),
        "phase_origin": phase_origin,
        "sequence": current_seq,
        "source": applied_source,
    }


def normalize_adc_sample_rate(sample_rate_hz):
    requested = int(round(float(sample_rate_hz)))
    return min(ADC_SAMPLE_RATES, key=lambda r: abs(r - requested))


def set_adc_rate(sample_rate_hz):
    global adc_transport_generation, next_bridge_poll_at, next_adc_counter

    requested = normalize_adc_sample_rate(sample_rate_hz)
    raw = str(call_bridge_control("set_adc_sample_rate", requested)).strip()
    parts = raw.split(",")
    if len(parts) != 2:
        raise ValueError(f"Bad ADC rate response: {raw!r}")

    actual = int(parts[0])
    counter = int(parts[1])
    if actual not in ADC_SAMPLE_RATES:
        raise ValueError(f"Unexpected ADC rate: {actual}")

    with data_lock:
        adc_running = bool(state["adc_running"])
        state["mcu_fs"] = float(actual)
        state["mcu_fs_nominal"] = float(actual)
        state["sample_budget_us"] = 1000000.0 / float(actual)
        state["adc_timer_rate_hz"] = actual
        state["latest_counter"] = counter
        state["phase_origin"] = counter
        state["adc_read_last_us"] = 0
        state["adc_read_max_us"] = 0
        state["scheduler_late_max_us"] = 0
        state["adc_conversion_timeouts"] = 0
        state["bridge_backlog_samples"] = 0
        state["bridge_backoff_ms"] = 0.0

        # Changing sample rate changes counter-to-time scaling. Start a clean
        # analysis epoch so old samples are never interpreted at the new rate.
        source_state["changed_counter"] = counter
        source_state["generation"] += 1
        if source_state["running"]:
            latch_source_for_analysis_locked()

        adc_transport_generation += 1
        current_seq = web_sequence

    clear_samples(counter if adc_running else None)
    next_bridge_poll_at = time.monotonic()
    set_bridge_state(True)

    return {
        "ok": True,
        "sample_rate_hz": actual,
        "counter": counter,
        "sequence": current_seq,
        "running": adc_running,
    }


def set_adc_run(running):
    global adc_transport_generation, next_bridge_poll_at, next_adc_counter

    counter = int(call_bridge_control("set_adc_run", 1 if running else 0))

    with data_lock:
        state["adc_running"] = bool(running)
        state["latest_counter"] = counter
        state["bridge_backlog_samples"] = 0
        state["bridge_backoff_ms"] = 0.0
        current_seq = web_sequence

        adc_transport_generation += 1

    if running:
        # RUN starts a fresh acquisition, as before.
        clear_samples(counter)
    else:
        # STOP is a true HOLD: preserve the server ring and cached MPU
        # measurement/FFT so the lower graph remains inspectable.
        with data_lock:
            next_adc_counter = None

    next_bridge_poll_at = time.monotonic()

    set_bridge_state(True)

    return {
        "ok": True,
        "running": bool(running),
        "counter": counter,
        "sequence": current_seq,
    }


ADC_PACK_ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_"
ADC_PACK_DECODE = {c: i for i, c in enumerate(ADC_PACK_ALPHABET)}


def decode_packed_adc14(payload, count):
    """Decode v18.8 2x14-bit -> 5 printable-char sample pairs.

    The old 4-hex-char/sample format remains accepted for compatibility.
    """
    pair_count = (int(count) + 1) // 2
    needed = pair_count * 5
    if len(payload) < needed:
        raise ValueError(
            f"Short packed ADC payload: expected {needed}, got {len(payload)}"
        )

    adc = []
    pos = 0
    for _ in range(pair_count):
        value = 0
        for _ in range(5):
            ch = payload[pos]
            pos += 1
            try:
                digit = ADC_PACK_DECODE[ch]
            except KeyError as exc:
                raise ValueError(f"Bad packed ADC character: {ch!r}") from exc
            value = (value << 6) | digit

        adc.append((value >> 14) & 0x3FFF)
        if len(adc) < count:
            adc.append(value & 0x3FFF)

    return adc


def parse_adc_batch(raw):
    text = str(raw)

    if "|" not in text:
        raise ValueError(f"Bad batch: {text!r}")

    header, payload = text.split("|", 1)
    parts = header.split(",")

    if len(parts) not in (4, 5, 8, 15):
        raise ValueError(f"Bad header: {header!r}")

    latest_counter = int(parts[0])
    phase_origin = int(parts[1])
    start_counter = int(parts[2])
    count = int(parts[3])

    sample_rate_hz = MCU_FS_NOMINAL
    if len(parts) >= 5:
        candidate = int(parts[4]) / 1000.0
        if 10.0 <= candidate <= 5000.0:
            sample_rate_hz = candidate

    adc_read_last_us = int(parts[5]) if len(parts) >= 8 else 0
    adc_read_max_us = int(parts[6]) if len(parts) >= 8 else 0
    scheduler_late_max_us = int(parts[7]) if len(parts) >= 8 else 0
    adc_dma_transfer_count = int(parts[8]) if len(parts) >= 15 else 0
    adc_dma_block_count = int(parts[9]) if len(parts) >= 15 else 0
    adc_dma_error_count = int(parts[10]) if len(parts) >= 15 else 0
    adc_overrun_count = int(parts[11]) if len(parts) >= 15 else 0
    adc_dma_service_last_us = int(parts[12]) if len(parts) >= 15 else 0
    adc_dma_service_max_us = int(parts[13]) if len(parts) >= 15 else 0
    adc_dma_last_status = int(parts[14]) if len(parts) >= 15 else 0

    if count < 0 or count > RPC_BATCH_SIZE:
        raise ValueError(f"Bad count: {count}")

    if payload.startswith("~"):
        adc = decode_packed_adc14(payload[1:], count)
    else:
        needed = count * 4
        if len(payload) < needed:
            raise ValueError(
                f"Short ADC payload: expected {needed}, got {len(payload)}"
            )

        adc = [
            int(payload[i:i + 4], 16)
            for i in range(0, needed, 4)
        ]

    return (
        latest_counter,
        phase_origin,
        start_counter,
        sample_rate_hz,
        adc_read_last_us,
        adc_read_max_us,
        scheduler_late_max_us,
        adc_dma_transfer_count,
        adc_dma_block_count,
        adc_dma_error_count,
        adc_overrun_count,
        adc_dma_service_last_us,
        adc_dma_service_max_us,
        adc_dma_last_status,
        adc,
    )


def format_uptime(seconds):
    total = max(0, int(seconds))
    days, rem = divmod(total, 86400)
    hours, rem = divmod(rem, 3600)
    minutes, secs = divmod(rem, 60)
    clock = f"{hours:02d}:{minutes:02d}:{secs:02d}"
    return f"{days}d {clock}" if days else clock


def network_diag_note_client(remote_ip, path):
    # Count active dashboard computers by source IP. On the normal UNO Q LAN,
    # one computer has one IP; multiple tabs on the same computer still count
    # as one computer, which is the useful quantity for classroom load tests.
    if path not in ("/api/samples", "/api/analysis"):
        return
    now = time.monotonic()
    with network_diag_lock:
        network_diag_client_last_seen[str(remote_ip)] = now


def network_diag_note_response(method, path, tx_bytes, duration_ms):
    global network_diag_total_requests, network_diag_total_tx_bytes
    now = time.monotonic()
    item = (now, str(method), str(path), int(tx_bytes), float(duration_ms))
    with network_diag_lock:
        network_diag_events.append(item)
        network_diag_total_requests += 1
        network_diag_total_tx_bytes += max(0, int(tx_bytes))

        # Keep only a short rolling history. This is diagnostic bookkeeping,
        # not a data logger, so memory stays bounded even during long runs.
        cutoff = now - max(NETWORK_DIAG_WINDOW_S, NETWORK_ACTIVE_CLIENT_WINDOW_S)
        while network_diag_events and network_diag_events[0][0] < cutoff:
            network_diag_events.popleft()

        stale = [
            key for key, seen_at in network_diag_client_last_seen.items()
            if seen_at < now - NETWORK_ACTIVE_CLIENT_WINDOW_S
        ]
        for key in stale:
            network_diag_client_last_seen.pop(key, None)


def network_diag_snapshot():
    now = time.monotonic()
    with network_diag_lock:
        cutoff = now - NETWORK_DIAG_WINDOW_S
        events = [e for e in network_diag_events if e[0] >= cutoff]
        active_clients = sum(
            1 for seen_at in network_diag_client_last_seen.values()
            if seen_at >= now - NETWORK_ACTIVE_CLIENT_WINDOW_S
        )
        total_requests = int(network_diag_total_requests)
        total_tx_bytes = int(network_diag_total_tx_bytes)

    elapsed = max(1.0, min(NETWORK_DIAG_WINDOW_S, now - network_diag_started_at))

    def matching(path=None, method=None):
        return [
            e for e in events
            if (path is None or e[2] == path) and (method is None or e[1] == method)
        ]

    def rate(rows):
        return len(rows) / elapsed

    def timing(rows):
        if not rows:
            return 0.0, 0.0
        vals = [e[4] for e in rows]
        return sum(vals) / len(vals), max(vals)

    samples_rows = matching(path="/api/samples")
    analysis_rows = matching(path="/api/analysis")
    status_rows = matching(path="/api/status")
    post_rows = matching(method="POST")

    http_avg, http_max = timing(events)
    samples_avg, samples_max = timing(samples_rows)
    analysis_avg, analysis_max = timing(analysis_rows)
    tx_bytes_window = sum(max(0, e[3]) for e in events)
    tx_bytes_per_s = tx_bytes_window / elapsed

    return {
        "active_clients": int(active_clients),
        "active_client_basis": "remote_ip",
        "active_client_window_s": NETWORK_ACTIVE_CLIENT_WINDOW_S,
        "window_s": NETWORK_DIAG_WINDOW_S,
        "http_rps": rate(events),
        "samples_rps": rate(samples_rows),
        "analysis_rps": rate(analysis_rows),
        "control_rps": rate(post_rows),
        "status_rps": rate(status_rows),
        "json_tx_kib_s": tx_bytes_per_s / 1024.0,
        "json_tx_mbps": (tx_bytes_per_s * 8.0) / 1000000.0,
        "http_response_avg_ms": http_avg,
        "http_response_max_ms": http_max,
        "samples_response_avg_ms": samples_avg,
        "samples_response_max_ms": samples_max,
        "analysis_response_avg_ms": analysis_avg,
        "analysis_response_max_ms": analysis_max,
        "total_http_requests": total_requests,
        "total_json_tx_bytes": total_tx_bytes,
    }


def quick_status_locked():
    uptime_seconds = max(0, int(time.monotonic() - APP_STARTED_AT))
    bridge = (
        "OK"
        if state["bridge_ok"]
        else ("CONNECTING" if state["bridge_connecting"] else "ERROR")
    )
    return {
        "bridge": bridge,
        "dac": "RUN" if state["dac_running"] else "STOP",
        "adc": "RUN" if state["adc_running"] else "STOP",
        "dac_rate": f"{state['dac_timer_rate_hz'] / 1000.0:g} kHz" if state["dac_timer_rate_hz"] else "—",
        "adc_rate": f"{state['mcu_fs'] / 1000.0:g} kS/s" if state["mcu_fs"] >= 1000 else f"{state['mcu_fs']:g} S/s",
        "noise": f"{'ON' if state['noise_enabled'] else 'OFF'} / {state['noise_level_percent']}%",
        "backlog": int(state["bridge_backlog_samples"]),
        "uptime": format_uptime(uptime_seconds),
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def _json(self, obj, status=200):
        raw = json.dumps(obj, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.end_headers()
        self.wfile.write(raw)

        started = getattr(self, "_diag_request_started_at", None)
        duration_ms = (time.perf_counter() - started) * 1000.0 if started else 0.0
        network_diag_note_response(
            getattr(self, "_diag_method", self.command),
            getattr(self, "_diag_path", urlparse(self.path).path),
            len(raw),
            duration_ms,
        )

    def _read_json(self):
        n = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(n) if n else b"{}"
        return json.loads(raw.decode("utf-8") or "{}")

    def do_GET(self):
        self._diag_request_started_at = time.perf_counter()
        self._diag_method = "GET"
        parsed = urlparse(self.path)
        self._diag_path = parsed.path
        network_diag_note_client(self.client_address[0], parsed.path)

        # The browser-facing URL remains stable: / or /index.html.
        # The old versioned HTML URLs are redirected once so an already-open
        # development tab lands on the stable address after the upgrade.
        if parsed.path.startswith("/unoq_scope_v") and parsed.path.endswith(".html"):
            self.send_response(302)
            self.send_header("Location", "/")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            return

        if parsed.path == "/api/status":
            with data_lock:
                result = {k: v for k, v in state.items() if not (k.startswith("dma_") or k.startswith("adc_dma_"))}
                result["sequence"] = web_sequence
                result["ring_size"] = samples.size
                result["source"] = source_snapshot_locked()
                result["ui"] = shared_ui_snapshot_locked()
                # Keep the non-DMA diagnostic status and append a small
                # human-readable summary at the very end for quick checks.
                result["quick_status"] = quick_status_locked()
            # v19.50: passive rolling HTTP/client diagnostics. This snapshot
            # uses its own tiny lock and does not touch the acquisition path.
            result["network"] = network_diag_snapshot()
            self._json(result)
            return

        if parsed.path == "/api/dac_diag":
            try:
                self._json({"ok": True, **read_dac_diag()})
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, status=503)
            return

        if parsed.path == "/api/samples":
            qs = parse_qs(parsed.query)

            try:
                after = int(qs.get("after", ["-1"])[0])
            except Exception:
                after = -1

            latest_only = qs.get("latest", ["0"])[0].lower() in ("1", "true", "yes")

            with data_lock:
                if latest_only:
                    rows = samples.read_latest(max_items=512)
                    missed = False
                else:
                    rows, missed = samples.read_after(after, max_items=512)

                # Browser LIVE transport only needs the fields consumed by
                # index.html. Keep detailed ADC/DMA/Bridge/analysis diagnostics
                # on /api/status instead of repeating the full state at LIVE rate.
                result = {
                    "ok": state["bridge_ok"],
                    "error": state["bridge_error"],
                    "bridge_connecting": state["bridge_connecting"],
                    "bridge_poll_failures": state["bridge_poll_failures"],
                    "mcu_fs": state["mcu_fs"],
                    "display_fps": state["display_fps"],
                    "adc_running": state["adc_running"],
                    "dac_running": state["dac_running"],
                    "source_synced": state["source_synced"],
                    "phase_origin": state["phase_origin"],
                    "latest_counter": state["latest_counter"],
                    "dropped_samples": state["dropped_samples"],
                    "sequence": web_sequence,
                    "ring_size": samples.size,
                    "client_missed": missed,
                    "source": source_snapshot_locked(),
                    "ui": shared_ui_snapshot_locked(),
                    "samples": rows,
                }

            self._json(result)
            return

        if parsed.path == "/api/analysis":
            qs = parse_qs(parsed.query)
            compact = qs.get("compact", ["0"])[0] in ("1", "true", "yes")

            result = analysis_engine.snapshot(compact=compact)

            with data_lock:
                result["bridge_ok"] = state["bridge_ok"]
                result["bridge_connecting"] = state["bridge_connecting"]
                result["bridge_error"] = state["bridge_error"]

            self._json(result)
            return

        rel = UI_FILE if parsed.path in ("/", "/index.html") else parsed.path.lstrip("/")
        target = (ASSETS / rel).resolve()

        try:
            target.relative_to(ASSETS.resolve())
        except ValueError:
            self.send_error(403)
            return

        if not target.is_file():
            self.send_error(404)
            return

        data = target.read_bytes()
        ctype, _ = mimetypes.guess_type(str(target))

        self.send_response(200)
        self.send_header("Content-Type", ctype or "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        self._diag_request_started_at = time.perf_counter()
        self._diag_method = "POST"
        parsed = urlparse(self.path)
        self._diag_path = parsed.path
        network_diag_note_client(self.client_address[0], parsed.path)

        try:
            payload = self._read_json()

            if parsed.path == "/api/config":
                self._json(apply_config(payload))
                return

            if parsed.path == "/api/noise":
                self._json(set_noise(
                    bool(payload.get("enabled", False)),
                    payload.get("level_percent", 30),
                ))
                return

            if parsed.path == "/api/dac/run":
                self._json(set_dac_run(bool(payload.get("running", True))))
                return

            if parsed.path == "/api/adc/run":
                running = bool(payload.get("running", True))
                result = set_adc_run(running)

                if not running:
                    analysis_engine.cancel_autoset("ADC acquisition stopped")

                self._json(result)
                return

            if parsed.path == "/api/adc/rate":
                result = set_adc_rate(payload.get("sample_rate_hz", MCU_FS_NOMINAL))
                analysis_engine.cancel_autoset("ADC sample rate changed")
                self._json(result)
                return

            if parsed.path == "/api/ui":
                with data_lock:
                    ui = update_shared_ui_locked(payload)
                self._json({"ok": True, "ui": ui})
                return

            if parsed.path == "/api/analysis/trigger":
                cfg = analysis_engine.configure_trigger(payload)
                # Trigger configuration was already server-authoritative.  Mirror
                # it into shared UI state so other browsers update their visible
                # controls without changing the existing trigger engine path.
                with data_lock:
                    ui = update_shared_ui_locked({
                        "trigger_enabled": cfg.get("enabled", False),
                        "trigger_edge": cfg.get("edge", "rise"),
                        "trigger_level_v": cfg.get("level_v", 0.0),
                        "trigger_config_generation": cfg.get("generation", 0),
                    })
                self._json({"ok": True, "trigger_config": cfg, "ui": ui})
                return

            if parsed.path == "/api/analysis/autoset":
                with data_lock:
                    adc_running = bool(state["adc_running"])
                    latest_counter = int(state["latest_counter"])
                    fs_hz = float(state["mcu_fs"])

                if not adc_running:
                    self._json({
                        "ok": False,
                        "error": "ADC STOPPED",
                    }, 409)
                    return

                self._json(
                    analysis_engine.arm_autoset(
                        latest_counter,
                        fs_hz,
                        time_div_ms=payload.get("time_div_ms"),
                    )
                )
                return

            self._json({"ok": False, "error": "not found"}, 404)

        except Exception as exc:
            set_bridge_state(False, str(exc))
            self._json({"ok": False, "error": str(exc)}, 500)


def analysis_worker():
    """MPU-side cached analysis.

    Important: the Bridge acquisition loop never performs FFT work.
    Snapshots are copied under data_lock, then all signal processing runs
    outside that lock in this separate Linux thread.
    """
    last_measure_at = 0.0
    last_fft_at = 0.0
    last_measure_seq = -1
    last_fft_seq = -1
    last_source_generation = -1

    while True:
        now = time.monotonic()

        measurement_snapshot = None
        fft_snapshot = None

        with data_lock:
            current_seq = web_sequence
            current_source = dict(source_state)

            acquisition = {
                "fs_hz": state["mcu_fs"],
                "nyquist_hz": state["mcu_fs"] / 2.0,
                "latest_counter": state["latest_counter"],
                "sequence": current_seq,
                "ring_size": samples.size,
                "transport_dropped_samples": state["dropped_samples"],
                "bridge_poll_mode": state["bridge_poll_mode"],
                "bridge_poll_period_ms": state["bridge_poll_period_ms"],
                "bridge_poll_interval_ms": state["bridge_poll_interval_ms"],
                "bridge_poll_duration_ms": state["bridge_poll_duration_ms"],
                "bridge_poll_late_ms": state["bridge_poll_late_ms"],
                "last_batch_count": state["last_batch_count"],
                "bridge_backlog_samples": state["bridge_backlog_samples"],
                "bridge_backoff_ms": state["bridge_backoff_ms"],
            }

            source_changed = (
                current_source["generation"] != last_source_generation
            )

            if (
                current_seq != last_measure_seq
                and now - last_measure_at >= MPU_MEASUREMENT_INTERVAL_S
            ):
                measurement_snapshot = samples.snapshot_latest(
                    MPU_MEASUREMENT_SAMPLES
                )

            if (
                measurement_snapshot is None
                and current_seq != last_fft_seq
                and now - last_fft_at >= MPU_FFT_INTERVAL_S
                and now - last_measure_at >= MPU_ANALYSIS_PHASE_OFFSET_S
            ):
                fft_snapshot = samples.snapshot_latest(
                    MPU_FFT_SAMPLES
                )

        # Refresh requested/source information immediately, even with ADC STOP.
        if source_changed:
            analysis_engine.refresh_source(current_source, acquisition)
            last_source_generation = current_source["generation"]

        if measurement_snapshot is not None:
            measure_started = time.perf_counter()
            measure_timing = analysis_engine.update_measurement(
                measurement_snapshot,
                current_source,
                acquisition,
            ) or {}
            measure_ms = (time.perf_counter() - measure_started) * 1000.0
            compare_ms = float(measure_timing.get("compare_ms", 0.0) or 0.0)
            with data_lock:
                state["measure_last_ms"] = measure_ms
                state["measure_max_ms"] = max(state["measure_max_ms"], measure_ms)
                state["compare_last_ms"] = compare_ms
                state["compare_max_ms"] = max(state["compare_max_ms"], compare_ms)
            last_measure_seq = current_seq
            last_measure_at = now

        if fft_snapshot is not None:
            fft_started = time.perf_counter()
            analysis_engine.update_fft(
                fft_snapshot,
                current_source,
                acquisition,
            )
            fft_ms = (time.perf_counter() - fft_started) * 1000.0
            with data_lock:
                state["fft_last_ms"] = fft_ms
                state["fft_max_ms"] = max(state["fft_max_ms"], fft_ms)
            last_fft_seq = current_seq
            last_fft_at = now

        time.sleep(0.05)


def http_thread():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"UNO Q dashboard listening on port {PORT}")
    server.serve_forever()


threading.Thread(target=http_thread, daemon=True).start()
threading.Thread(target=analysis_worker, daemon=True).start()


def loop():
    global web_sequence, next_adc_counter, next_bridge_poll_at
    global last_bridge_poll_started_at, mcu_state_synced
    global next_initial_sync_attempt_at

    if not mcu_state_synced:
        # App install/flash can start Linux well before the MCU RouterBridge
        # endpoint is usable. An early blocking Bridge.call() has proven able
        # to wedge the first session until a board reboot, so v19.0 deliberately
        # gives the freshly-flashed MCU a long quiet boot window. Failed first
        # sync attempts are then spaced out instead of immediately hammering
        # another 10-second RPC.
        now_sync = time.monotonic()
        if now_sync < next_initial_sync_attempt_at:
            time.sleep(min(0.10, next_initial_sync_attempt_at - now_sync))
            return
        try:
            sync_source_from_mcu()
            next_bridge_poll_at = time.monotonic()
        except Exception as exc:
            mark_bridge_poll_failure(str(exc))
            next_initial_sync_attempt_at = time.monotonic() + BRIDGE_INITIAL_RETRY_S
            time.sleep(0.20)
            return

    now = time.monotonic()

    with data_lock:
        adc_running = bool(state["adc_running"])
        poll_generation = adc_transport_generation
        requested_counter = next_adc_counter
        prior_failures = int(state["bridge_poll_failures"])

    # ADC STOP: only a low-rate Bridge health check.
    poll_interval = (
        BRIDGE_NORMAL_POLL_S if adc_running else BRIDGE_STOPPED_POLL_S
    )

    if now < next_bridge_poll_at:
        time.sleep(min(next_bridge_poll_at - now, 0.05))
        return

    # Give explicit DAC/ADC/config commands priority.
    if control_pending.is_set():
        next_bridge_poll_at = time.monotonic() + BRIDGE_CONTROL_YIELD_S
        time.sleep(BRIDGE_CONTROL_YIELD_S)
        return

    if requested_counter is None:
        # STOP state or a just-reset transport. Use the latest known counter
        # only for the health-check call; do not consume old samples.
        with data_lock:
            requested_counter = int(state["latest_counter"])

    poll_started = time.monotonic()

    if last_bridge_poll_started_at is None:
        poll_interval_ms = 0.0
    else:
        poll_interval_ms = (
            poll_started - last_bridge_poll_started_at
        ) * 1000.0

    last_bridge_poll_started_at = poll_started

    try:
        hi = (int(requested_counter) >> 16) & 0xFFFF
        lo = int(requested_counter) & 0xFFFF

        bridge_started = time.monotonic()
        raw = call_bridge_poll("get_adc_since", hi, lo)

        if raw is None:
            next_bridge_poll_at = time.monotonic() + BRIDGE_CONTROL_YIELD_S
            return

        bridge_duration_ms = (time.monotonic() - bridge_started) * 1000.0

        (
            latest_counter,
            phase_origin,
            start_counter,
            sample_rate_hz,
            adc_read_last_us,
            adc_read_max_us,
            scheduler_late_max_us,
            adc_dma_transfer_count,
            adc_dma_block_count,
            adc_dma_error_count,
            adc_overrun_count,
            adc_dma_service_last_us,
            adc_dma_service_max_us,
            adc_dma_last_status,
            adc_values,
        ) = parse_adc_batch(raw)

        mark_bridge_poll_success()

        with data_lock:
            # Discard a response that crossed an ADC RUN/STOP reset.
            if poll_generation != adc_transport_generation:
                next_bridge_poll_at = time.monotonic() + BRIDGE_CONTROL_YIELD_S
                return

            state["phase_origin"] = phase_origin
            state["latest_counter"] = latest_counter
            state["mcu_fs"] = float(sample_rate_hz)
            state["mcu_fs_nominal"] = float(sample_rate_hz)
            state["sample_budget_us"] = 1000000.0 / max(1.0, float(sample_rate_hz))
            state["adc_read_last_us"] = int(adc_read_last_us)
            state["adc_read_max_us"] = int(adc_read_max_us)
            state["scheduler_late_max_us"] = int(scheduler_late_max_us)
            state["adc_dma_transfer_count"] = int(adc_dma_transfer_count)
            state["adc_dma_block_count"] = int(adc_dma_block_count)
            state["adc_dma_error_count"] = int(adc_dma_error_count)
            state["adc_overrun_count"] = int(adc_overrun_count)
            state["adc_dma_service_last_us"] = int(adc_dma_service_last_us)
            state["adc_dma_service_max_us"] = int(adc_dma_service_max_us)
            state["adc_dma_last_status"] = int(adc_dma_last_status)
            state["bridge_poll_interval_ms"] = poll_interval_ms
            state["bridge_poll_duration_ms"] = bridge_duration_ms
            state["bridge_poll_late_ms"] = 0.0
            state["last_batch_count"] = len(adc_values)
            state["bridge_backoff_ms"] = 0.0

            if adc_running:
                requested = int(next_adc_counter)

                # A jump means the MPU fell behind the 2048-sample MCU history.
                if start_counter > requested:
                    state["dropped_samples"] += start_counter - requested

                # Older-than-requested data is never appended.
                skip = max(0, requested - start_counter)

                for i in range(skip, len(adc_values)):
                    sample_counter = start_counter + i
                    web_sequence += 1

                    samples.append(
                        web_sequence,
                        sample_counter,
                        adc_values[i],
                        phase_origin,
                    )

                next_adc_counter = start_counter + len(adc_values)
                backlog = max(0, latest_counter - next_adc_counter)
                state["bridge_backlog_samples"] = backlog
            else:
                backlog = 0
                state["bridge_backlog_samples"] = 0

        # Adaptive schedule:
        # - backlog: immediately request the next unseen chunk
        # - normal: maintain ~25 ms start-to-start pacing
        # - ADC STOP: only one health check per second
        if adc_running and backlog > 0:
            next_bridge_poll_at = time.monotonic()
        else:
            next_bridge_poll_at = poll_started + poll_interval
            if next_bridge_poll_at < time.monotonic():
                next_bridge_poll_at = time.monotonic()

    except Exception as exc:
        mark_bridge_poll_failure(str(exc))

        with data_lock:
            failures = max(1, int(state["bridge_poll_failures"]))

        # Circuit-breaker style cooldown: do not hammer RouterBridge while it
        # is already unhealthy.
        backoff = min(
            BRIDGE_BACKOFF_MAX_S,
            0.10 * (2 ** min(failures - 1, 3)),
        )

        with data_lock:
            state["bridge_backoff_ms"] = backoff * 1000.0

        next_bridge_poll_at = time.monotonic() + backoff
        time.sleep(min(backoff, 0.10))


App.run(user_loop=loop)
