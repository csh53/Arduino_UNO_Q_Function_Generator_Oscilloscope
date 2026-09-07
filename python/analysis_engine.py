import math
import threading
import time

try:
    import numpy as _np
except Exception:
    _np = None

FFT_BACKEND = "numpy" if _np is not None else "python_radix2"


ADC_FULL_SCALE = 16383.0
ADC_VREF = 3.3

FFT_TARGET_N = 16384
# acquisition["fs_hz"] is the MCU-measured wall-clock sample rate.
# The measured MCU rate remains the analysis clock; v19.11 production acquisition is fixed at 2000 S/s.
MEASUREMENT_MAX_N = 8192
TIME_COMPARE_MAX_N = 1024
SPECTRUM_MAX_HZ = 10000.0
SPECTRUM_DB_FLOOR = -60.0

TRIGGER_MAX_N = 32768
TRIGGER_SEARCH_MAX_N = 2048
TRIGGER_HYSTERESIS_MIN_V = 0.010
TRIGGER_HYSTERESIS_MAX_V = 0.080
TRIGGER_HYSTERESIS_FRACTION = 0.03
AUTOSET_MAX_N = 512
AUTOSET_TARGET_SAMPLES = 512
AUTOSET_MAX_WAIT_MS = 500.0
AUTOSET_LOW_FREQ_DIRECT_HZ = 5.0
AUTOSET_TRIGGER_MIN_VISIBLE_PERIODS = 1.25
TIME_DIVS_MS = [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000]
PERIODIC_MIN_VPP = 0.004
PERIODIC_HYSTERESIS_MIN_V = 0.0008
PERIODIC_HYSTERESIS_MAX_V = 0.025
PERIODIC_HYSTERESIS_FRACTION = 0.02
PERIOD_CV_MAX = 0.20
SPECTRAL_PROMINENCE_MIN_DB = 9.0

# v17.5 low-frequency frequency lock.
# Below/at 20 Hz, a clean Schmitt rising-edge period is a stronger physical
# time-domain cue than requiring FFT/crossing agreement on every update.
LOW_FREQ_EDGE_MAX_HZ = 20.5
LOW_FREQ_PERIOD_CV_MAX = 0.12
LOW_FREQ_PERIOD_OUTLIER_FRACTION = 0.22
LOW_FREQ_PERIOD_OUTLIER_MIN_SAMPLES = 1.5
LOW_FREQ_MIN_INLIER_PERIODS = 2
LOW_FREQ_STRONG_EDGE_PERIODS = 3
LOW_FREQ_MIN_PROMINENCE_DB = 6.0
# Sparse high-frequency edge lock for the 2 kS/s production mode.  Between
# roughly 200 and 500 Hz there are only 10..4 samples/cycle, so FFT-bin
# agreement can be unnecessarily conservative even when the time-domain edge
# train is clean.  Require several regular periods and stay comfortably below
# Nyquist.
HIGH_FREQ_EDGE_MIN_HZ = 80.0
HIGH_FREQ_EDGE_MAX_FS_FRACTION = 0.45
HIGH_FREQ_PERIOD_CV_MAX = 0.26
HIGH_FREQ_MIN_PERIODS = 5
HIGH_FREQ_MIN_PROMINENCE_DB = 5.0
NOISE_FLOOR_MAX_VPP = 0.050
OSC_MIN_VERTICAL_SPAN_V = 0.050
COMPARISON_CORRELATION_MIN = 0.90
RELATION_ACQUIRE_COUNT = 3
RELATION_RELEASE_COUNT = 8


def _finite_or_none(value):
    try:
        value = float(value)
    except Exception:
        return None
    return value if math.isfinite(value) else None


def _wrap_phase_deg(value):
    if value is None:
        return None
    while value > 180.0:
        value -= 360.0
    while value <= -180.0:
        value += 360.0
    return value


def _highest_power_of_two(n):
    n = int(n)
    if n < 1:
        return 0
    return 1 << (n.bit_length() - 1)


def _raw_to_v(code):
    return float(code) * ADC_VREF / ADC_FULL_SCALE


def _amp_to_dbv(amp_vpk):
    # Match the existing dashboard convention:
    # 20 log10(Vpk / 1 V).
    return 20.0 * math.log10(max(float(amp_vpk), 1e-12))


def _waveform_unit(phase, waveform):
    phase = phase % 1.0
    if waveform == "sine":
        return math.sin(2.0 * math.pi * phase)
    if waveform == "square":
        return 1.0 if phase < 0.5 else -1.0
    if waveform == "triangle":
        return 1.0 - 4.0 * abs(phase - 0.5)
    return 0.0


def _reference_value(counter, phase_origin, source, fs_hz):
    if not source.get("running", False):
        return 0.0

    waveform = source.get("waveform", "dc")
    offset_v = float(source.get("offset_v", 1.65))

    if waveform == "dc":
        return max(0.0, min(3.3, offset_v))

    freq_hz = float(source.get("frequency_hz", 10.0))
    amplitude_v = float(source.get("amplitude_v", 1.0))
    phase = (float(counter) - float(phase_origin)) * freq_hz / float(fs_hz)

    v = offset_v + amplitude_v * _waveform_unit(phase, waveform)
    return max(0.0, min(3.3, v))



def _periodicity_metrics(signal_v, fs_hz):
    n = len(signal_v)
    result = {
        "periodic": False,
        "classification": "not_ready",
        "frequency_hz": None,
        "frequency_method": None,
        "crossing_frequency_hz": None,
        "crossing_periods": 0,
        "crossing_periods_raw": 0,
        "period_cv": None,
        "spectral_peak_hz": None,
        "spectral_prominence_db": None,
        "frequency_match": False,
    }

    if n < 32:
        return result

    min_v = min(signal_v)
    max_v = max(signal_v)
    vpp_v = max_v - min_v
    level = (min_v + max_v) / 2.0

    # Robust rising crossings.
    #
    # A slow waveform such as a 1 Hz triangle can spend many ADC samples
    # close to the midpoint. ADC noise can then create several false
    # midpoint crossings and make a 1 Hz signal look like 5 Hz or more.
    # Arm below a lower threshold and accept one rising crossing only after
    # reaching the upper threshold (Schmitt-style hysteresis).
    hysteresis_v = max(
        PERIODIC_HYSTERESIS_MIN_V,
        min(
            PERIODIC_HYSTERESIS_MAX_V,
            PERIODIC_HYSTERESIS_FRACTION * vpp_v,
        ),
    )
    low_level = level - hysteresis_v
    high_level = level + hysteresis_v

    crossings = []
    armed = signal_v[0] <= low_level
    prev = signal_v[0]

    for i in range(1, n):
        cur = signal_v[i]

        if not armed:
            if cur <= low_level:
                armed = True
        elif cur >= high_level:
            # Interpolate around the actual midpoint when possible. For an
            # abrupt square-wave edge, the same interpolation still works.
            denom = cur - prev
            if prev < level <= cur and abs(denom) >= 1e-15:
                frac = (level - prev) / denom
            else:
                frac = 0.0

            crossings.append(
                (i - 1) + max(0.0, min(1.0, frac))
            )
            armed = False

        prev = cur

    periods = []
    for i in range(1, len(crossings)):
        p = crossings[i] - crossings[i - 1]
        if p >= 2.0:
            periods.append(p)

    result["crossing_periods_raw"] = len(periods)

    crossing_freq = None
    period_cv = None
    period_inliers = []

    if periods:
        # Robust period estimate:
        # a missed/double edge creates an isolated ~0.5x / ~2x period.  A
        # median-centered inlier set rejects those without biasing a clean
        # low-frequency square/sine measurement.
        ordered = sorted(periods)
        m = len(ordered)
        if m & 1:
            median_period = ordered[m // 2]
        else:
            median_period = 0.5 * (
                ordered[m // 2 - 1] + ordered[m // 2]
            )

        tolerance_samples = max(
            LOW_FREQ_PERIOD_OUTLIER_MIN_SAMPLES,
            LOW_FREQ_PERIOD_OUTLIER_FRACTION * median_period,
        )

        period_inliers = [
            p for p in periods
            if abs(p - median_period) <= tolerance_samples
        ]

        # For a very short acquisition, retain the available periods rather
        # than throwing the whole measurement away.
        if len(period_inliers) < LOW_FREQ_MIN_INLIER_PERIODS:
            period_inliers = ordered[1:-1] if len(ordered) >= 5 else ordered

        avg = sum(period_inliers) / len(period_inliers)

        if avg > 0:
            crossing_freq = float(fs_hz) / avg
            variance = sum(
                (p - avg) ** 2 for p in period_inliers
            ) / len(period_inliers)
            period_cv = math.sqrt(variance) / avg if avg else None

    result["crossing_periods"] = len(period_inliers)
    result["crossing_frequency_hz"] = crossing_freq
    result["period_cv"] = period_cv

    peak_hz = None
    prominence_db = None
    df_hz = None

    nfft = min(4096, _highest_power_of_two(n))

    if nfft >= 256 and _np is not None:
        arr = _np.asarray(signal_v[-nfft:], dtype=_np.float64)
        arr = arr - arr.mean()
        window = _np.hanning(nfft)
        spec = _np.abs(_np.fft.rfft(arr * window))

        if len(spec) > 1:
            spec[0] = 0.0
            peak_bin = int(_np.argmax(spec))
            peak_amp = float(spec[peak_bin])
            df_hz = float(fs_hz) / nfft
            peak_hz = peak_bin * df_hz

            mask = _np.ones(len(spec), dtype=bool)
            lo = max(0, peak_bin - 2)
            hi = min(len(spec), peak_bin + 3)
            mask[lo:hi] = False
            mask[0] = False
            floor_values = spec[mask]

            if floor_values.size:
                floor_amp = float(_np.median(floor_values))
                prominence_db = 20.0 * math.log10(
                    max(peak_amp, 1e-18) / max(floor_amp, 1e-18)
                )

    result["spectral_peak_hz"] = peak_hz
    result["spectral_prominence_db"] = prominence_db

    freq_match = False
    if crossing_freq and peak_hz is not None and df_hz is not None:
        tolerance = max(2.0 * df_hz, 0.08 * crossing_freq)
        freq_match = abs(crossing_freq - peak_hz) <= tolerance

    result["frequency_match"] = freq_match

    edge_regular = bool(
        vpp_v >= PERIODIC_MIN_VPP
        and crossing_freq is not None
        and len(period_inliers) >= LOW_FREQ_MIN_INLIER_PERIODS
        and period_cv is not None
        and period_cv <= LOW_FREQ_PERIOD_CV_MAX
    )

    low_freq_candidate = bool(
        edge_regular
        and crossing_freq <= LOW_FREQ_EDGE_MAX_HZ
        and crossing_freq < 0.45 * float(fs_hz)
    )

    spectral_support = bool(
        prominence_db is not None
        and prominence_db >= LOW_FREQ_MIN_PROMINENCE_DB
    )

    # A clean low-frequency edge train should not be rejected merely because
    # the FFT peak temporarily lands on a neighboring bin/harmonic.  Require
    # either spectral support or at least three highly regular measured
    # periods.  This keeps random/noisy crossings from becoming a false lock.
    low_freq_edge_lock = bool(
        low_freq_candidate
        and (
            spectral_support
            or len(period_inliers) >= LOW_FREQ_STRONG_EDGE_PERIODS
        )
    )

    # At 2 kS/s, a 200..500 Hz waveform has only about 10..4 samples/cycle.
    # The interpolated edge periods are still a strong timing cue, while the
    # FFT peak can hop between neighboring bins/harmonics.  Accept a clean,
    # regular edge train in this sparse-sampling region without requiring exact
    # FFT agreement.
    high_freq_edge_lock = bool(
        vpp_v >= PERIODIC_MIN_VPP
        and crossing_freq is not None
        and crossing_freq >= HIGH_FREQ_EDGE_MIN_HZ
        and crossing_freq < HIGH_FREQ_EDGE_MAX_FS_FRACTION * float(fs_hz)
        and len(period_inliers) >= HIGH_FREQ_MIN_PERIODS
        and period_cv is not None
        and period_cv <= HIGH_FREQ_PERIOD_CV_MAX
        and (
            (prominence_db is not None and prominence_db >= HIGH_FREQ_MIN_PROMINENCE_DB)
            or freq_match
            or len(period_inliers) >= 8
        )
    )

    if _np is not None:
        standard_lock = bool(
            vpp_v >= PERIODIC_MIN_VPP
            and len(period_inliers) >= 2
            and period_cv is not None
            and period_cv <= PERIOD_CV_MAX
            and prominence_db is not None
            and prominence_db >= SPECTRAL_PROMINENCE_MIN_DB
            and freq_match
        )
    else:
        standard_lock = bool(
            vpp_v >= PERIODIC_MIN_VPP
            and len(period_inliers) >= 3
            and period_cv is not None
            and period_cv <= 0.12
        )

    periodic = bool(low_freq_edge_lock or high_freq_edge_lock or standard_lock)

    if periodic:
        result["periodic"] = True
        result["classification"] = "periodic"
        result["frequency_hz"] = crossing_freq
        result["frequency_method"] = (
            "edge_low_freq"
            if low_freq_edge_lock
            else ("edge_sparse_high_freq" if high_freq_edge_lock else "crossing_fft")
        )
    else:
        result["classification"] = (
            "noise_floor"
            if vpp_v <= NOISE_FLOOR_MAX_VPP
            else "nonperiodic"
        )

    return result



def _best_effort_measurement_metrics(signal_v, fs_hz):
    """Fail-fast LIVE frequency measurement.

    v19.21 deliberately stops after a small number of usable edge periods.
    A clean waveform locks quickly; a noisy/ambiguous waveform is rejected
    after the same bounded sample of periods instead of collecting and sorting
    hundreds or thousands of pseudo-crossings.  Low-frequency signals still
    get the full available time window because only a few periods exist.
    """
    n = len(signal_v)
    result = {
        "periodic": False,
        "classification": "not_ready",
        "frequency_hz": None,
        "frequency_method": None,
        "crossing_frequency_hz": None,
        "crossing_periods": 0,
        "crossing_periods_raw": 0,
        "period_cv": None,
        "spectral_peak_hz": None,
        "spectral_prominence_db": None,
        "frequency_match": False,
    }

    if n < 48 or fs_hz <= 0:
        return result

    min_v = min(signal_v)
    max_v = max(signal_v)
    vpp_v = max_v - min_v
    if vpp_v < PERIODIC_MIN_VPP:
        result["classification"] = "noise_floor"
        return result

    level = 0.5 * (min_v + max_v)
    hysteresis_v = max(0.003, min(0.080, 0.05 * vpp_v))
    low_level = level - hysteresis_v
    high_level = level + hysteresis_v

    # Eight periods are enough to identify an obvious LIVE edge train.  This
    # bound is the key fail-fast rule for noisy/high-crossing inputs.
    target_periods = 8
    periods = []
    last_crossing = None
    armed = signal_v[0] <= low_level
    prev = signal_v[0]

    for i in range(1, n):
        cur = signal_v[i]
        if not armed:
            if cur <= low_level:
                armed = True
        elif cur >= high_level:
            denom = cur - prev
            if prev < level <= cur and abs(denom) >= 1e-15:
                frac = (level - prev) / denom
            else:
                frac = 0.0
            crossing = (i - 1) + max(0.0, min(1.0, frac))
            armed = False

            if last_crossing is not None:
                period = crossing - last_crossing
                if period >= 2.0:
                    periods.append(period)
                    if len(periods) >= target_periods:
                        break
            last_crossing = crossing
        prev = cur

    result["crossing_periods_raw"] = len(periods)
    if len(periods) < 3:
        result["classification"] = "nonperiodic"
        return result

    # Sorting at most eight values is bounded and negligible.  It preserves the
    # useful outlier rejection of earlier versions without unbounded work.
    ordered = sorted(periods)
    m = len(ordered)
    median_period = (
        ordered[m // 2]
        if m & 1
        else 0.5 * (ordered[m // 2 - 1] + ordered[m // 2])
    )
    tolerance = max(1.0, 0.14 * median_period)
    inliers = [p for p in periods if abs(p - median_period) <= tolerance]
    inlier_ratio = len(inliers) / len(periods)

    if len(inliers) < 3 or inlier_ratio < 0.70:
        result["classification"] = "nonperiodic"
        return result

    avg = sum(inliers) / len(inliers)
    variance = sum((p - avg) ** 2 for p in inliers) / len(inliers)
    period_cv = math.sqrt(variance) / avg if avg > 0 else None
    crossing_freq = float(fs_hz) / avg if avg > 0 else None

    result["crossing_periods"] = len(inliers)
    result["crossing_frequency_hz"] = crossing_freq
    result["period_cv"] = period_cv

    periodic = bool(
        crossing_freq is not None
        and 0.05 <= crossing_freq < 0.45 * float(fs_hz)
        and period_cv is not None
        and period_cv <= 0.10
        and (len(inliers) >= 4 or crossing_freq <= 20.5)
    )

    if periodic:
        result.update({
            "periodic": True,
            "classification": "periodic",
            "frequency_hz": crossing_freq,
            "frequency_method": "edge_fail_fast",
        })
    else:
        result["classification"] = "nonperiodic"

    return result


def _best_effort_measurement_metrics_numpy(signal_np, fs_hz):
    """Vectorized LIVE measurement path used only by MEASURE.

    v19.26 keeps the v19.21 fail-fast decision rules but avoids converting the
    measurement window back to a Python list.  Threshold event discovery,
    period differences and inlier statistics remain in NumPy.  AUTOSET keeps
    its existing small-window implementation unchanged.
    """
    result = {
        "periodic": False,
        "classification": "not_ready",
        "frequency_hz": None,
        "frequency_method": None,
        "crossing_frequency_hz": None,
        "crossing_periods": 0,
        "crossing_periods_raw": 0,
        "period_cv": None,
        "spectral_peak_hz": None,
        "spectral_prominence_db": None,
        "frequency_match": False,
    }

    if _np is None:
        return _best_effort_measurement_metrics(signal_np, fs_hz)

    arr = _np.asarray(signal_np, dtype=_np.float64)
    n = int(arr.size)
    if n < 48 or fs_hz <= 0:
        return result

    min_v = float(_np.min(arr))
    max_v = float(_np.max(arr))
    vpp_v = max_v - min_v
    if vpp_v < PERIODIC_MIN_VPP:
        result["classification"] = "noise_floor"
        return result

    level = 0.5 * (min_v + max_v)
    hysteresis_v = max(0.003, min(0.080, 0.05 * vpp_v))
    low_level = level - hysteresis_v
    high_level = level + hysteresis_v

    # Vectorized Schmitt arm/re-arm state.  At each sample, the detector is
    # armed exactly when the most recent LOW-threshold event is newer than the
    # most recent HIGH-threshold event.  A HIGH sample in that state is the
    # accepted rising edge.  This is O(N) NumPy work with no Python sample loop
    # and no event sorting.
    sample_idx = _np.arange(n, dtype=_np.int64)
    low_mask = arr <= low_level
    high_mask = arr >= high_level
    if not _np.any(low_mask) or not _np.any(high_mask):
        result["classification"] = "nonperiodic"
        return result

    last_low = _np.maximum.accumulate(_np.where(low_mask, sample_idx, -1))
    last_high = _np.maximum.accumulate(_np.where(high_mask, sample_idx, -1))
    prev_low = _np.empty_like(last_low)
    prev_high = _np.empty_like(last_high)
    prev_low[0] = -1
    prev_high[0] = -1
    prev_low[1:] = last_low[:-1]
    prev_high[1:] = last_high[:-1]

    crossing_idx = _np.flatnonzero(
        high_mask & (prev_low > prev_high)
    ).astype(_np.int64, copy=False)
    if crossing_idx.size < 4:
        result["classification"] = "nonperiodic"
        return result

    # Eight periods (nine crossings) preserve the bounded fail-fast behavior.
    crossing_idx = crossing_idx[:9]

    # Match the previous midpoint interpolation rule.  When the Schmitt HIGH
    # event occurs after the midpoint was already crossed, the old code used
    # i-1 rather than searching backward, so preserve that behavior.
    prev_idx = _np.maximum(crossing_idx - 1, 0)
    prev_v = arr[prev_idx]
    cur_v = arr[crossing_idx]
    denom = cur_v - prev_v
    can_interp = (prev_v < level) & (cur_v >= level) & (_np.abs(denom) >= 1e-15)
    frac = _np.zeros(crossing_idx.size, dtype=_np.float64)
    frac[can_interp] = (level - prev_v[can_interp]) / denom[can_interp]
    frac = _np.clip(frac, 0.0, 1.0)
    crossings = (crossing_idx.astype(_np.float64) - 1.0) + frac

    periods = _np.diff(crossings)
    periods = periods[periods >= 2.0]
    if periods.size > 8:
        periods = periods[:8]

    result["crossing_periods_raw"] = int(periods.size)
    if periods.size < 3:
        result["classification"] = "nonperiodic"
        return result

    median_period = float(_np.median(periods))
    tolerance = max(1.0, 0.14 * median_period)
    inliers = periods[_np.abs(periods - median_period) <= tolerance]
    inlier_ratio = float(inliers.size) / float(periods.size)

    if inliers.size < 3 or inlier_ratio < 0.70:
        result["classification"] = "nonperiodic"
        return result

    avg = float(_np.mean(inliers))
    period_cv = float(_np.std(inliers) / avg) if avg > 0 else None
    crossing_freq = float(fs_hz) / avg if avg > 0 else None

    result["crossing_periods"] = int(inliers.size)
    result["crossing_frequency_hz"] = crossing_freq
    result["period_cv"] = period_cv

    periodic = bool(
        crossing_freq is not None
        and 0.05 <= crossing_freq < 0.45 * float(fs_hz)
        and period_cv is not None
        and period_cv <= 0.10
        and (int(inliers.size) >= 4 or crossing_freq <= 20.5)
    )

    if periodic:
        result.update({
            "periodic": True,
            "classification": "periodic",
            "frequency_hz": crossing_freq,
            "frequency_method": "edge_fail_fast",
        })
    else:
        result["classification"] = "nonperiodic"

    return result


def _estimate_frequency(signal_v, fs_hz):
    quality = _periodicity_metrics(signal_v, fs_hz)
    return quality["frequency_hz"], quality["crossing_periods"]


def _fft_in_place(values):
    """Iterative radix-2 complex FFT, standard-library only."""
    n = len(values)
    j = 0

    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit

        if i < j:
            values[i], values[j] = values[j], values[i]

    length = 2
    while length <= n:
        angle = -2.0 * math.pi / length
        w_len = complex(math.cos(angle), math.sin(angle))
        half = length >> 1

        for start in range(0, n, length):
            w = 1.0 + 0.0j
            stop = start + half

            for i in range(start, stop):
                u = values[i]
                v = values[i + half] * w
                values[i] = u + v
                values[i + half] = u - v
                w *= w_len

        length <<= 1


def _hann_fft(signal_v, fs_hz):
    available = len(signal_v)
    n = min(FFT_TARGET_N, _highest_power_of_two(available))

    if n < 256:
        return {
            "ready": False,
            "target_n": FFT_TARGET_N,
            "n": 0,
            "window": "hann",
            "backend": FFT_BACKEND,
            "df_hz": None,
            "bins": 0,
            "measured_peak_hz": None,
            "measured_peak_dbv": None,
            "magnitude_dbv": [],
        }

    signal_v = signal_v[-n:]
    df_hz = float(fs_hz) / n

    if _np is not None:
        arr = _np.asarray(signal_v, dtype=_np.float64)
        arr = arr - arr.mean()
        window = _np_hann_window(n)
        sum_w = float(window.sum())
        spec = _np.fft.rfft(arr * window)

        amps = _np.abs(spec) * (2.0 / sum_w)
        dbv = 20.0 * _np.log10(_np.maximum(amps, 1e-12))
        dbv = _np.maximum(dbv, -160.0)

        if len(amps) > 1:
            peak_bin = int(_np.argmax(amps[1:])) + 1
            peak_amp = float(amps[peak_bin])
        else:
            peak_bin = None
            peak_amp = -1.0

        magnitudes_dbv = _np.round(dbv, 4).tolist()

    else:
        mean_v = sum(signal_v) / n
        denom = max(1, n - 1)

        work = [0j] * n
        sum_w = 0.0

        for i, sample in enumerate(signal_v):
            w = 0.5 - 0.5 * math.cos(2.0 * math.pi * i / denom)
            work[i] = complex((sample - mean_v) * w, 0.0)
            sum_w += w

        _fft_in_place(work)

        bins = n // 2 + 1
        scale = 2.0 / sum_w if sum_w > 0 else 0.0
        magnitudes_dbv = [0.0] * bins
        peak_amp = -1.0
        peak_bin = None

        for k in range(bins):
            amp = abs(work[k]) * scale
            dbv = _amp_to_dbv(amp)
            magnitudes_dbv[k] = round(max(-160.0, dbv), 4)

            if k > 0 and amp > peak_amp:
                peak_amp = amp
                peak_bin = k

    bins = n // 2 + 1
    peak_hz = peak_bin * df_hz if peak_bin is not None else None
    peak_dbv = _amp_to_dbv(peak_amp) if peak_amp > 0 else None

    return {
        "ready": True,
        "target_n": FFT_TARGET_N,
        "n": n,
        "window": "hann",
        "backend": FFT_BACKEND,
        "df_hz": df_hz,
        "bins": bins,
        "measured_peak_hz": peak_hz,
        "measured_peak_dbv": peak_dbv,
        "magnitude_dbv": magnitudes_dbv,
    }


# Small NumPy caches used by the exact-tone transfer analysis.  The DAC/ADC
# settings normally stay fixed for many FFT updates, so reusing the Hann window
# and complex oscillator avoids rebuilding thousands of sin/cos values every
# second.  The cache is deliberately bounded because the frequency slider may
# be moved through many values during a session.
_NP_HANN_CACHE = {}
_NP_TONE_BASIS_CACHE = {}
_NP_TONE_BASIS_ORDER = []
_NP_TONE_BASIS_CACHE_MAX = 32


def _np_hann_window(n):
    window = _NP_HANN_CACHE.get(int(n))
    if window is None:
        window = _np.hanning(int(n)).astype(_np.float64, copy=False)
        _NP_HANN_CACHE[int(n)] = window
    return window


def _np_tone_basis(n, freq_hz, fs_hz):
    # Rounded keys keep numerically equivalent float settings on one cache key.
    key = (int(n), round(float(freq_hz), 9), round(float(fs_hz), 6))
    basis = _NP_TONE_BASIS_CACHE.get(key)
    if basis is not None:
        return basis

    idx = _np.arange(int(n), dtype=_np.float64)
    angle = (-2.0j * math.pi * float(freq_hz) / float(fs_hz)) * idx
    basis = _np.exp(angle)

    _NP_TONE_BASIS_CACHE[key] = basis
    _NP_TONE_BASIS_ORDER.append(key)
    if len(_NP_TONE_BASIS_ORDER) > _NP_TONE_BASIS_CACHE_MAX:
        old_key = _NP_TONE_BASIS_ORDER.pop(0)
        _NP_TONE_BASIS_CACHE.pop(old_key, None)
    return basis


def _exact_tones(signal_v, frequencies_hz, fs_hz):
    """Extract a bounded set of exact-frequency tones.

    v19.22 uses NumPy vector operations when NumPy is present (the production
    FFT backend), while preserving the old scalar implementation as a fallback.
    The signal is centered/windowed once, then each requested tone is a single
    vector dot product instead of an N-sample Python sin/cos loop.
    """
    n = len(signal_v)
    freqs = [float(f) for f in frequencies_hz]
    result = {}

    if n < 8:
        return {f: None for f in freqs}

    valid = [f for f in freqs if 0.0 < f < float(fs_hz) / 2.0]
    for f in freqs:
        if f not in valid:
            result[f] = None

    if not valid:
        return result

    if _np is not None:
        arr = _np.asarray(signal_v, dtype=_np.float64)
        window = _np_hann_window(n)
        sum_w = float(window.sum())
        if sum_w <= 0.0:
            for f in valid:
                result[f] = None
            return result

        centered_windowed = (arr - float(arr.mean())) * window
        scale = 2.0 / sum_w

        for f in valid:
            coeff = _np.dot(centered_windowed, _np_tone_basis(n, f, fs_hz))
            coeff = complex(coeff)
            result[f] = {
                "amp_vpk": scale * abs(coeff),
                "phase_rad": math.atan2(coeff.imag, coeff.real),
            }
        return result

    # Scalar fallback preserves the pre-v19.22 behavior on environments where
    # NumPy is unavailable.
    mean_v = sum(signal_v) / n
    denom = max(1, n - 1)
    window = [
        0.5 - 0.5 * math.cos(2.0 * math.pi * i / denom)
        for i in range(n)
    ]
    sum_w = sum(window)
    if sum_w <= 0.0:
        for f in valid:
            result[f] = None
        return result

    centered_windowed = [
        (float(signal_v[i]) - mean_v) * window[i]
        for i in range(n)
    ]

    for f in valid:
        re = 0.0
        im = 0.0
        step = 2.0 * math.pi * f / float(fs_hz)
        for i, x in enumerate(centered_windowed):
            angle = step * i
            re += x * math.cos(angle)
            im -= x * math.sin(angle)
        result[f] = {
            "amp_vpk": 2.0 * math.hypot(re, im) / sum_w,
            "phase_rad": math.atan2(im, re),
        }
    return result


def _exact_tone(signal_v, freq_hz, fs_hz):
    # Compatibility wrapper for any future single-tone callers.
    freq_hz = float(freq_hz)
    return _exact_tones(signal_v, [freq_hz], fs_hz).get(freq_hz)


def _reference_values(counters, origins, source, fs_hz):
    """Generate the virtual CH1 DAC reference without Python per-sample loops.

    CH1 is not FFT-analyzed: it is a known mathematical DAC reference.  NumPy
    simply evaluates that known waveform for the sample counters in one vector
    operation so transfer phase/gain remain aligned to the real ADC window.
    """
    n = min(len(counters), len(origins))
    if n <= 0:
        return _np.asarray([], dtype=_np.float64) if _np is not None else []

    if _np is None:
        return [
            _reference_value(counters[i], origins[i], source, fs_hz)
            for i in range(n)
        ]

    if not source.get("running", False):
        return _np.zeros(n, dtype=_np.float64)

    waveform = str(source.get("waveform", "dc"))
    offset_v = float(source.get("offset_v", 1.65))
    if waveform == "dc":
        return _np.full(n, max(0.0, min(3.3, offset_v)), dtype=_np.float64)

    freq_hz = float(source.get("frequency_hz", 10.0))
    amplitude_v = float(source.get("amplitude_v", 1.0))
    ctr = _np.asarray(counters[:n], dtype=_np.float64)
    org = _np.asarray(origins[:n], dtype=_np.float64)
    phase = _np.remainder((ctr - org) * (freq_hz / float(fs_hz)), 1.0)

    if waveform == "sine":
        unit = _np.sin(2.0 * math.pi * phase)
    elif waveform == "square":
        unit = _np.where(phase < 0.5, 1.0, -1.0)
    elif waveform == "triangle":
        unit = 1.0 - 4.0 * _np.abs(phase - 0.5)
    else:
        unit = _np.zeros(n, dtype=_np.float64)

    return _np.clip(offset_v + amplitude_v * unit, 0.0, 3.3)



def _single_lag_correlation(ref, adc, mean_ref, mean_adc, lag):
    """One Pearson-style correlation at one already-known/estimated lag.

    v19.20 performs no lag sweep.  Phase at the known DAC fundamental gives a
    single best-effort lag; correlation is evaluated once at that location.
    """
    n = min(len(ref), len(adc))
    if n < 8:
        return None

    lag = int(lag)
    start = 0 if lag >= 0 else -lag
    end = n - lag if lag >= 0 else n
    if end - start < 8:
        return None

    sum_ra = 0.0
    sum_r2 = 0.0
    sum_a2 = 0.0

    for i in range(start, end):
        r = ref[i] - mean_ref
        a = adc[i + lag] - mean_adc
        sum_ra += r * a
        sum_r2 += r * r
        sum_a2 += a * a

    if sum_r2 <= 1e-18 or sum_a2 <= 1e-18:
        return None

    return sum_ra / math.sqrt(sum_r2 * sum_a2)


def _known_frequency_phase(signal_v, mean_v, freq_hz, fs_hz):
    """Estimate only the known source-frequency phase in O(N).

    The oscillator is advanced recursively, so LIVE comparison needs just one
    sin/cos pair per call instead of searching many lags.  This is intentionally
    best-effort: unrelated external ADC signals may yield a mathematically valid
    but physically meaningless phase, which the operator can simply ignore.
    """
    n = len(signal_v)
    if n < 8 or freq_hz <= 0.0 or freq_hz >= fs_hz / 2.0:
        return None

    omega = 2.0 * math.pi * freq_hz / fs_hz
    cw = math.cos(omega)
    sw = math.sin(omega)
    c = 1.0
    si = 0.0
    re = 0.0
    im = 0.0

    for sample in signal_v:
        x = sample - mean_v
        re += x * c
        im -= x * si
        c, si = c * cw - si * sw, si * cw + c * sw

    if abs(re) <= 1e-18 and abs(im) <= 1e-18:
        return None

    return math.atan2(im, re)


def _time_comparison(snapshot, source, acquisition, measurement_periodicity=None):
    raw = snapshot.get("adc", [])
    counters = snapshot.get("counter", [])
    origins = snapshot.get("origin", [])

    total = min(len(raw), len(counters), len(origins))
    result = {
        "valid": False,
        "relation_valid": False,
        "relation_reason": "ADC time-domain data not ready",
        "reason": "ADC time-domain data not ready",
        "n": 0,
        "window_s": None,
        "mean_error_v": None,
        "rmse_v": None,
        "peak_error_v": None,
        "delay_ms": None,
        "phase_deg": None,
        "correlation": None,
        "lag_samples": None,
        "measured_frequency_hz": None,
        "continuity": {
            "received_samples": 0,
            "counter_span": 0,
            "missing_samples": 0,
            "sample_loss_percent": 0.0,
            "effective_fs_hz": float(acquisition["fs_hz"]),
            "analysis_valid": False,
            "reason": "ADC time-domain data not ready",
        },
    }

    if not source.get("running", False):
        result["reason"] = "DAC not running"
        result["relation_reason"] = result["reason"]
        return result

    if total < 32:
        result["reason"] = "Not enough ADC samples"
        result["relation_reason"] = result["reason"]
        return result

    changed_counter = int(source.get("changed_counter", 0) or 0)

    start = 0
    while start < total and int(counters[start]) < changed_counter:
        start += 1

    available = total - start
    if available < 32:
        result["reason"] = "Waiting for post-change source samples"
        result["relation_reason"] = result["reason"]
        return result

    n = min(available, TIME_COMPARE_MAX_N)
    start = total - n

    raw = raw[start:total]
    counters = counters[start:total]
    origins = origins[start:total]

    continuity = _continuity_from_counters(counters, acquisition["fs_hz"])
    result["continuity"] = continuity
    result["n"] = n
    result["window_s"] = n / float(acquisition["fs_hz"])

    if not continuity["analysis_valid"]:
        result["reason"] = continuity["reason"]
        result["relation_reason"] = result["reason"]
        return result

    adc = [_raw_to_v(v) for v in raw]
    ref = [
        _reference_value(counters[i], origins[i], source, acquisition["fs_hz"])
        for i in range(n)
    ]

    mean_error = 0.0
    mse = 0.0
    peak_error = 0.0
    mean_adc = 0.0
    mean_ref = 0.0

    for i in range(n):
        err = adc[i] - ref[i]
        mean_error += err
        mse += err * err
        peak_error = max(peak_error, abs(err))
        mean_adc += adc[i]
        mean_ref += ref[i]

    mean_error /= n
    mse /= n
    mean_adc /= n
    mean_ref /= n

    result["mean_error_v"] = mean_error
    result["rmse_v"] = math.sqrt(mse)
    result["peak_error_v"] = peak_error

    waveform = str(source.get("waveform", "dc"))
    freq_hz = float(source.get("frequency_hz", 0.0) or 0.0)

    # v19.20 comparison policy: do not decide whether the wiring is a loopback.
    # Always do one bounded same-window calculation from the known DAC reference.
    # With unrelated external ADC input, the numbers may simply be meaningless.
    if waveform == "dc" or freq_hz <= 0.0:
        result.update({
            "valid": True,
            "relation_valid": True,
            "relation_reason": "",
            "reason": "",
        })
        return result

    result["measured_frequency_hz"] = freq_hz

    phase_ref = _known_frequency_phase(
        ref, mean_ref, freq_hz, float(acquisition["fs_hz"])
    )
    phase_adc = _known_frequency_phase(
        adc, mean_adc, freq_hz, float(acquisition["fs_hz"])
    )

    phase_deg = None
    delay_ms = None
    lag_samples = None
    lag_for_corr = 0

    if phase_ref is not None and phase_adc is not None:
        phase_deg = _wrap_phase_deg(
            (phase_adc - phase_ref) * 180.0 / math.pi
        )
        delay_ms = -phase_deg / (360.0 * freq_hz) * 1000.0
        lag_samples = delay_ms / 1000.0 * float(acquisition["fs_hz"])
        lag_for_corr = int(round(lag_samples))

    # One alignment check only.  No -lag..+lag sweep, refinement or lock hunt.
    corr = _single_lag_correlation(
        ref, adc, mean_ref, mean_adc, lag_for_corr
    )

    relation_valid = bool(corr is not None)
    result.update({
        "valid": relation_valid,
        "relation_valid": relation_valid,
        "relation_reason": "" if relation_valid else "Comparison unavailable",
        "reason": "" if relation_valid else "Comparison unavailable",
        "delay_ms": delay_ms,
        "phase_deg": phase_deg,
        "correlation": corr,
        "lag_samples": lag_samples,
    })

    return result


def _nearest_time_div_ms(freq_hz):
    if freq_hz is None or freq_hz <= 0:
        return None

    target = 250.0 / float(freq_hz)
    best = TIME_DIVS_MS[0]
    best_error = float("inf")

    for value in TIME_DIVS_MS:
        err = abs(math.log(float(value) / max(target, 1e-12)))
        if err < best_error:
            best_error = err
            best = value

    return float(best)


def _trigger_analysis(snapshot, acquisition, config):
    """Bounded Schmitt trigger diagnostic.

    The browser performs the live display alignment locally.  This MPU copy is
    retained for status/diagnostics, but it is intentionally bounded and never
    invokes FFT, periodicity or correlation work.
    """
    raw = snapshot.get("adc", [])
    counters = snapshot.get("counter", [])
    total = min(len(raw), len(counters))

    enabled = bool(config.get("enabled", True))
    edge = "fall" if str(config.get("edge", "rise")) == "fall" else "rise"
    level_v = max(0.0, min(3.3, float(config.get("level_v", 0.0))))
    position = max(0.05, min(0.95, float(config.get("position", 0.20))))
    requested_n = max(2, int(config.get("window_samples", 160)))
    n = min(requested_n, TRIGGER_MAX_N)

    result = {
        "available": True,
        "enabled": enabled,
        "edge": edge,
        "level_v": level_v,
        "position": position,
        "window_samples": n,
        "locked": False,
        "trigger_counter": None,
        "trigger_fraction": 0.0,
        "trigger_index": max(1, int(n * position)),
        "status": "OFF · FREE RUN" if not enabled else "AUTO · FREE RUN",
        "config_generation": int(config.get("generation", 0)),
    }

    if not enabled or total < 8 or n < 8:
        return result

    pre = max(1, int(n * position))
    post = n - pre - 1
    result["trigger_index"] = pre
    latest_candidate = total - post - 1
    earliest_candidate = max(1, pre)
    if latest_candidate < earliest_candidate:
        return result

    # Search only a bounded recent region.  Trigger is an always-on service,
    # so one pass must remain cheap even when the input is pure noise.
    search_start = max(
        earliest_candidate,
        latest_candidate - TRIGGER_SEARCH_MAX_N + 1,
    )

    # Estimate a local span for adaptive hysteresis using at most the same
    # bounded region.  The band suppresses chatter around the trigger level.
    span_start = max(0, search_start - 1)
    span_stop = min(total, latest_candidate + 1)
    local_codes = raw[span_start:span_stop]
    if local_codes:
        local_min = _raw_to_v(min(local_codes))
        local_max = _raw_to_v(max(local_codes))
        local_vpp = max(0.0, local_max - local_min)
    else:
        local_vpp = 0.0

    hyst = max(
        TRIGGER_HYSTERESIS_MIN_V,
        min(
            TRIGGER_HYSTERESIS_MAX_V,
            TRIGGER_HYSTERESIS_FRACTION * local_vpp,
        ),
    )
    low = level_v - hyst
    high = level_v + hyst

    found = -1
    frac = 0.0

    if edge == "rise":
        armed = _raw_to_v(raw[search_start - 1]) <= low
        prev = _raw_to_v(raw[search_start - 1])
        for i in range(search_start, latest_candidate + 1):
            cur = _raw_to_v(raw[i])
            if not armed:
                if cur <= low:
                    armed = True
            elif cur >= high:
                found = i
                denom = cur - prev
                if prev < level_v <= cur and abs(denom) > 1e-15:
                    frac = max(0.0, min(1.0, (level_v - prev) / denom))
                else:
                    frac = 0.0
                armed = False
            prev = cur
    else:
        armed = _raw_to_v(raw[search_start - 1]) >= high
        prev = _raw_to_v(raw[search_start - 1])
        for i in range(search_start, latest_candidate + 1):
            cur = _raw_to_v(raw[i])
            if not armed:
                if cur >= high:
                    armed = True
            elif cur <= low:
                found = i
                denom = cur - prev
                if prev > level_v >= cur and abs(denom) > 1e-15:
                    frac = max(0.0, min(1.0, (level_v - prev) / denom))
                else:
                    frac = 0.0
                armed = False
            prev = cur

    if found < 0:
        return result

    result.update({
        "locked": True,
        "trigger_counter": int(counters[found]),
        "trigger_fraction": float(frac),
        "status": (
            f"AUTO · {'↑' if edge == 'rise' else '↓'} "
            f"{level_v:.2f} V · LOCKED"
        ),
        "hysteresis_v": float(hyst),
    })
    return result



def _measurement_frequency_candidate(measurement):
    """ADC-only frequency candidate used by AUTOSET."""
    if not measurement or not measurement.get("ready", False):
        return None

    freq_hz = measurement.get("frequency_hz")

    if measurement.get("periodic") and freq_hz is not None:
        try:
            f = float(freq_hz)
        except (TypeError, ValueError):
            f = 0.0

        if math.isfinite(f) and f > 0.0:
            return f

    # A slow clean waveform can occasionally have a valid strong FFT peak
    # even when the crossing-consistency test is momentarily conservative.
    try:
        peak_hz = float(measurement.get("spectral_peak_hz"))
        prominence_db = float(
            measurement.get("spectral_prominence_db")
        )
        vpp_v = float(measurement.get("vpp_v"))
    except (TypeError, ValueError):
        return None

    if (
        math.isfinite(peak_hz)
        and peak_hz > 0.0
        and prominence_db >= SPECTRAL_PROMINENCE_MIN_DB
        and vpp_v > NOISE_FLOOR_MAX_VPP
    ):
        return peak_hz

    return None


def _current_measurement_recommendation(
    measurement,
    freq_hz,
    trigger_enabled,
):
    if not measurement:
        return None

    min_v = measurement.get("min_v")
    max_v = measurement.get("max_v")

    if min_v is None or max_v is None:
        return None

    lo, hi = _adc_vertical_window(min_v, max_v)
    mid_v = (float(min_v) + float(max_v)) / 2.0
    vpp_v = measurement.get("vpp_v")
    if vpp_v is None:
        vpp_v = float(max_v) - float(min_v)
    trigger_edge = "rise" if trigger_enabled is not None else None
    trigger_stats = {
        "mid_v": mid_v,
        "vpp_v": vpp_v,
    }

    return {
        "vertical_lo_v": lo,
        "vertical_hi_v": hi,
        "trigger_edge": trigger_edge,
        "trigger_enabled": (bool(trigger_enabled) if trigger_enabled is not None else None),
        "trigger_level_v": (
            _autoset_trigger_level(trigger_stats, trigger_edge)
            if trigger_enabled
            else None
        ),
        "frequency_hz": (
            float(freq_hz)
            if freq_hz is not None
            else None
        ),
        "time_div_ms": _nearest_time_div_ms(freq_hz),
        "periodic": bool(freq_hz is not None),
        "classification": (
            "periodic"
            if freq_hz is not None
            else "nonperiodic"
        ),
    }



def _autoset_fresh_stats(snapshot, start_counter, fs_hz):
    raw = snapshot.get("adc", [])
    counters = snapshot.get("counter", [])
    total = min(len(raw), len(counters))

    if total <= 0:
        return None

    first = total
    for i in range(total - 1, -1, -1):
        if int(counters[i]) <= int(start_counter):
            first = i + 1
            break
        first = i

    available = total - first
    if available <= 0:
        return None

    n = min(available, AUTOSET_MAX_N)
    first = total - n

    signal_v = [_raw_to_v(v) for v in raw[first:total]]
    min_v = min(signal_v)
    max_v = max(signal_v)
    mean_v = sum(signal_v) / n
    mid_v = (min_v + max_v) / 2.0
    vpp_v = max_v - min_v
    quality = _best_effort_measurement_metrics(signal_v, fs_hz)

    return {
        "n": n,
        "min_v": min_v,
        "max_v": max_v,
        "mean_v": mean_v,
        "mid_v": mid_v,
        "vpp_v": vpp_v,
        "frequency_hz": quality["frequency_hz"],
        "frequency_method": quality["frequency_method"],
        "crossing_frequency_hz": quality["crossing_frequency_hz"],
        "crossing_periods": quality["crossing_periods"],
        "periodic": quality["periodic"],
        "classification": quality["classification"],
        "period_cv": quality["period_cv"],
        "spectral_peak_hz": quality["spectral_peak_hz"],
        "spectral_prominence_db": quality["spectral_prominence_db"],
        "frequency_match": quality["frequency_match"],
    }



def _adc_vertical_window(min_v, max_v, min_span=OSC_MIN_VERTICAL_SPAN_V):
    """Return a display viewport centered on the measured CH2 waveform.

    The ADC hardware range remains 0..3.3 V.  AUTOSET is only choosing the
    *display* viewport, so it may extend slightly below 0 V or above 3.3 V.
    Keeping equal headroom above and below the measured midpoint prevents
    near-rail signals from being visually pushed away from screen center.
    """
    raw_lo = float(min_v)
    raw_hi = float(max_v)
    if raw_hi < raw_lo:
        raw_lo, raw_hi = raw_hi, raw_lo

    center = (raw_lo + raw_hi) / 2.0
    raw_span = max(0.0, raw_hi - raw_lo)
    span = max(min_span, raw_span * 1.30, 0.004)

    lo = center - span / 2.0
    hi = center + span / 2.0
    return lo, hi


def _autoset_trigger_level(stats, edge="rise"):
    """Place AUTOSET trigger just beyond the waveform midpoint.

    A small 0.5%-of-Vpp offset avoids sitting exactly on the midpoint
    while still scaling naturally for small and large signals.
    """
    mid_v = float(stats["mid_v"])
    offset_v = 0.005 * max(0.0, float(stats.get("vpp_v", 0.0)))
    if str(edge).lower() == "fall":
        return mid_v - offset_v
    return mid_v + offset_v


def _autoset_recommendation(stats):
    if not stats:
        return None

    lo, hi = _adc_vertical_window(stats["min_v"], stats["max_v"])
    periodic = bool(stats.get("periodic"))
    freq_hz = stats.get("frequency_hz") if periodic else None
    trigger_edge = "rise" if periodic else None

    return {
        "vertical_lo_v": lo,
        "vertical_hi_v": hi,
        "trigger_edge": trigger_edge,
        # None means "leave the user's current trigger state untouched".
        "trigger_enabled": True if periodic else None,
        "trigger_level_v": (
            _autoset_trigger_level(stats, trigger_edge) if periodic else None
        ),
        "frequency_hz": freq_hz,
        "time_div_ms": _nearest_time_div_ms(freq_hz),
        "periodic": periodic,
        "classification": stats.get("classification", "nonperiodic"),
    }


def _analysis_display_source(source):
    """Return the source represented by the lower analysis graph.

    Live acquisition/comparison still uses source["running"].  The display
    source may remain available after DAC STOP via held_* fields.
    """
    available = bool(
        source.get("analysis_available", source.get("running", False))
    )

    if not available:
        return {
            "available": False,
            "hold": False,
            "waveform": str(source.get("waveform", "dc")),
            "frequency_hz": float(source.get("frequency_hz", 10.0)),
            "amplitude_v": float(source.get("amplitude_v", 1.0)),
            "offset_v": float(source.get("offset_v", 1.65)),
        }

    return {
        "available": True,
        "hold": bool(source.get("hold", False)),
        "waveform": str(
            source.get("held_waveform", source.get("waveform", "dc"))
        ),
        "frequency_hz": float(
            source.get("held_frequency_hz", source.get("frequency_hz", 10.0))
        ),
        "amplitude_v": float(
            source.get("held_amplitude_v", source.get("amplitude_v", 1.0))
        ),
        "offset_v": float(
            source.get("held_offset_v", source.get("offset_v", 1.65))
        ),
    }


def _requested_source_payload(source):
    display = _analysis_display_source(source)
    return {
        "running": bool(source.get("running", False)),
        "available": bool(display["available"]),
        "hold": bool(display["hold"]),
        "waveform": display["waveform"],
        "frequency_hz": float(display["frequency_hz"]),
        "amplitude_v": float(display["amplitude_v"]),
        "offset_v": float(display["offset_v"]),
        "generation": int(source.get("generation", 0)),
        "changed_counter": int(source.get("changed_counter", 0)),
    }

def _continuity_from_counters(counters, fs_hz):
    """Validate that the MPU received one ADC sample for every MCU sample tick."""
    received = len(counters)

    if received == 0:
        return {
            "received_samples": 0,
            "counter_span": 0,
            "missing_samples": 0,
            "sample_loss_percent": 0.0,
            "effective_fs_hz": None,
            "analysis_valid": False,
            "reason": "No ADC samples",
        }

    if received == 1:
        return {
            "received_samples": 1,
            "counter_span": 1,
            "missing_samples": 0,
            "sample_loss_percent": 0.0,
            "effective_fs_hz": float(fs_hz),
            "analysis_valid": True,
            "reason": "",
        }

    span = int(counters[-1]) - int(counters[0]) + 1

    if span <= 0:
        return {
            "received_samples": received,
            "counter_span": span,
            "missing_samples": received,
            "sample_loss_percent": 100.0,
            "effective_fs_hz": None,
            "analysis_valid": False,
            "reason": "Non-monotonic ADC counters",
        }

    missing = max(0, span - received)
    loss_percent = 100.0 * missing / span
    effective_fs = float(fs_hz) * received / span

    sequential = all(
        int(counters[i]) == int(counters[i - 1]) + 1
        for i in range(1, received)
    )
    valid = (missing == 0 and sequential)

    return {
        "received_samples": received,
        "counter_span": span,
        "missing_samples": missing,
        "sample_loss_percent": loss_percent,
        "effective_fs_hz": effective_fs,
        "analysis_valid": valid,
        "reason": "" if valid else "ADC sample gap detected",
    }


class MPUAnalysisEngine:
    """Cached signal-analysis engine for the UNO Q Linux MPU.

    The browser is not required for any calculation performed here.  The first
    migration revision keeps the old browser analysis active for side-by-side
    verification, while this engine runs independently in the background.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._trigger_config = {
            "enabled": False,
            "edge": "rise",
            "level_v": 0.0,
            "position": 0.20,
            "window_samples": 160,
            "generation": 0,
        }
        self._relation_state = {
            "locked": False,
            "good_count": 0,
            "bad_count": 0,
            "source_generation": -1,
        }
        self._autoset = {
            "request_id": 0,
            "active": False,
            "status": "idle",
            "start_counter": 0,
            "armed_monotonic": 0.0,
            "hint_freq": None,
            "min_samples": 256,
            "max_wait_ms": 500.0,
        }
        self._result = {
            "ok": True,
            "engine": "uno_q_mpu_python",
            "analysis_ready": False,
            "updated_unix_s": None,
            "acquisition": {
                "fs_hz": 200,
                "nyquist_hz": 100.0,
                "latest_counter": 0,
                "sequence": 0,
                "ring_size": 0,
                "continuity": {
                    "received_samples": 0,
                    "counter_span": 0,
                    "missing_samples": 0,
                    "sample_loss_percent": 0.0,
                    "effective_fs_hz": None,
                    "analysis_valid": False,
                    "reason": "No ADC samples",
                },
            },
            "measurement": {
                "ready": False,
                "n": 0,
                "latest_adc_code": None,
                "latest_v": None,
                "mean_v": None,
                "min_v": None,
                "max_v": None,
                "vpp_v": None,
                "rms_v": None,
                "frequency_hz": None,
                "frequency_method": None,
                "crossing_frequency_hz": None,
                "period_s": None,
                "crossing_periods": 0,
                "periodic": False,
                "signal_class": "not_ready",
                "period_cv": None,
                "spectral_peak_hz": None,
                "spectral_prominence_db": None,
            },
            "time_comparison": {
                "valid": False,
                "relation_valid": False,
                "relation_reason": "ADC time-domain data not ready",
                "reason": "ADC time-domain data not ready",
                "n": 0,
                "window_s": None,
                "mean_error_v": None,
                "rmse_v": None,
                "peak_error_v": None,
                "delay_ms": None,
                "phase_deg": None,
                "correlation": None,
                "lag_samples": None,
            },
            "trigger": {
                "available": True,
                "enabled": False,
                "edge": "rise",
                "level_v": 0.0,
                "position": 0.20,
                "window_samples": 160,
                "locked": False,
                "trigger_counter": None,
                "trigger_fraction": 0.0,
                "trigger_index": 32,
                "status": "OFF · FREE RUN",
                "config_generation": 0,
            },
            "autoset": {
                "request_id": 0,
                "active": False,
                "status": "idle",
                "complete": False,
                "reason": "",
                "fresh_samples": 0,
                "target_samples": 0,
                "elapsed_ms": 0.0,
                "stats": None,
                "recommendation": None,
            },
            "fft": {
                "ready": False,
                "target_n": FFT_TARGET_N,
                "n": 0,
                "window": "hann",
                "df_hz": None,
                "bins": 0,
                "measured_peak_hz": None,
                "measured_peak_dbv": None,
                "magnitude_dbv": [],
            },
            "requested": {
                "running": False,
                "available": False,
                "hold": False,
                "waveform": "dc",
                "frequency_hz": 10.0,
                "amplitude_v": 1.0,
                "offset_v": 1.65,
                "generation": 0,
                "changed_counter": 0,
            },
            "transfer": {
                "valid": False,
                "reason": "DAC not running",
                "gain_db": None,
                "phase_deg": None,
                "thd_percent": None,
            },
        }

    def configure_trigger(self, payload):
        with self._lock:
            old = dict(self._trigger_config)
            cfg = dict(old)
            cfg["enabled"] = bool(payload.get("enabled", cfg["enabled"]))
            cfg["edge"] = (
                "fall"
                if str(payload.get("edge", cfg["edge"])) == "fall"
                else "rise"
            )
            cfg["level_v"] = max(
                0.0,
                min(3.3, float(payload.get("level_v", cfg["level_v"]))),
            )
            cfg["position"] = max(
                0.05,
                min(0.95, float(payload.get("position", cfg["position"]))),
            )
            cfg["window_samples"] = max(
                2,
                min(
                    TRIGGER_MAX_N,
                    int(payload.get("window_samples", cfg["window_samples"])),
                ),
            )

            changed = not (
                cfg["enabled"] == old["enabled"]
                and cfg["edge"] == old["edge"]
                and abs(cfg["level_v"] - old["level_v"]) < 1e-9
                and abs(cfg["position"] - old["position"]) < 1e-9
                and cfg["window_samples"] == old["window_samples"]
            )
            if changed:
                cfg["generation"] = int(old.get("generation", 0)) + 1
                self._trigger_config = cfg
            else:
                cfg["generation"] = int(old.get("generation", 0))

            out = dict(cfg)
            out["changed"] = changed
            return out


    def arm_autoset(self, latest_counter, fs_hz, time_div_ms=None):
        with self._lock:
            current_measurement = dict(
                self._result.get("measurement", {})
            )
            request_id = int(self._autoset["request_id"]) + 1

        candidate_freq = _measurement_frequency_candidate(
            current_measurement
        )

        # If LIVE MEASURE already has a confident frequency, AUTOSET can
        # finish immediately from that cached result.  No fresh search or
        # trigger-lock wait is needed; the one-shot simply applies a sensible
        # timebase/vertical fit and arms the lightweight trigger once.
        if candidate_freq is not None:
            visible_periods = 2.5
            trigger_enabled = True

            recommendation = _current_measurement_recommendation(
                current_measurement,
                candidate_freq,
                trigger_enabled,
            )

            status = "locked" if trigger_enabled is True else "vertical_fit"

            result = {
                "request_id": request_id,
                "active": False,
                "status": status,
                "complete": True,
                "reason": "",
                "fresh_samples": 0,
                "target_samples": 0,
                "max_wait_ms": 0.0,
                "elapsed_ms": 0.0,
                "seed_frequency_hz": float(candidate_freq),
                "visible_periods": float(visible_periods),
                "stats": {
                    "n": int(current_measurement.get("n", 0) or 0),
                    "min_v": current_measurement.get("min_v"),
                    "max_v": current_measurement.get("max_v"),
                    "mean_v": current_measurement.get("mean_v"),
                    "mid_v": (
                        (
                            float(current_measurement["min_v"])
                            + float(current_measurement["max_v"])
                        ) / 2.0
                        if current_measurement.get("min_v") is not None
                        and current_measurement.get("max_v") is not None
                        else None
                    ),
                    "vpp_v": current_measurement.get("vpp_v"),
                    "frequency_hz": float(candidate_freq),
                    "periodic": True,
                    "classification": "periodic",
                    "period_cv": current_measurement.get("period_cv"),
                    "spectral_peak_hz": current_measurement.get(
                        "spectral_peak_hz"
                    ),
                    "spectral_prominence_db": current_measurement.get(
                        "spectral_prominence_db"
                    ),
                },
                "recommendation": recommendation,
            }

            with self._lock:
                self._autoset = {
                    "request_id": request_id,
                    "active": False,
                    "status": status,
                    "start_counter": int(latest_counter),
                    "armed_monotonic": time.monotonic(),
                    "min_samples": 0,
                    "max_wait_ms": 0.0,
                    "seed_frequency_hz": float(candidate_freq),
                }
                self._result["autoset"] = result

            return {
                "ok": True,
                "request_id": request_id,
                "start_counter": int(latest_counter),
                "target_samples": 0,
                "max_wait_ms": 0.0,
                "seed_frequency_hz": float(candidate_freq),
                "visible_periods": float(visible_periods),
                "direct": True,
            }

        # Normal AUTOSET remains short and bounded.  High-frequency signals do
        # not need a 1024-sample fresh capture: at 200..500 Hz, 512 samples
        # already contain dozens of periods.  Reducing the fresh-capture target
        # lowers lock latency and Bridge/analysis pressure without weakening
        # the low-frequency path.
        autoset_target = AUTOSET_TARGET_SAMPLES
        if candidate_freq is not None and candidate_freq >= 100.0:
            autoset_target = 512

        with self._lock:
            self._autoset = {
                "request_id": request_id,
                "active": True,
                "status": "acquiring",
                "start_counter": int(latest_counter),
                "armed_monotonic": time.monotonic(),
                "min_samples": autoset_target,
                "max_wait_ms": AUTOSET_MAX_WAIT_MS,
                "seed_frequency_hz": candidate_freq,
            }
            self._result["autoset"] = {
                "request_id": request_id,
                "active": True,
                "status": "acquiring",
                "complete": False,
                "reason": "",
                "fresh_samples": 0,
                "target_samples": autoset_target,
                "max_wait_ms": AUTOSET_MAX_WAIT_MS,
                "elapsed_ms": 0.0,
                "seed_frequency_hz": candidate_freq,
                "stats": None,
                "recommendation": None,
            }

        return {
            "ok": True,
            "request_id": request_id,
            "start_counter": int(latest_counter),
            "target_samples": autoset_target,
            "max_wait_ms": AUTOSET_MAX_WAIT_MS,
            "seed_frequency_hz": candidate_freq,
            "direct": False,
        }

    def cancel_autoset(self, reason="cancelled"):
        with self._lock:
            self._autoset["active"] = False
            current = dict(self._result.get("autoset", {}))
            current.update({
                "active": False,
                "status": "cancelled",
                "complete": True,
                "reason": str(reason),
                "recommendation": None,
            })
            self._result["autoset"] = current


    def _update_autoset(self, snapshot, source, acquisition):
        with self._lock:
            cfg = dict(self._autoset)

        if not cfg.get("active", False):
            return

        elapsed_ms = (
            time.monotonic() - float(cfg["armed_monotonic"])
        ) * 1000.0
        stats = _autoset_fresh_stats(
            snapshot,
            cfg["start_counter"],
            acquisition["fs_hz"],
        )
        fresh_samples = int(stats["n"]) if stats else 0
        enough_samples = fresh_samples >= int(cfg["min_samples"])
        timed_out = elapsed_ms >= float(cfg["max_wait_ms"])

        # One-shot best effort: complete on the first bounded fresh window.
        # If the waveform is obvious, set vertical/timebase/trigger. If it is
        # ambiguous, apply only the vertical fit and leave timebase/trigger as-is.
        complete = bool(enough_samples or timed_out)
        status = "acquiring"
        reason = ""
        recommendation = None

        if complete:
            if stats:
                recommendation = _autoset_recommendation(stats)
                if stats.get("periodic"):
                    status = "locked"
                else:
                    status = "vertical_fit"
                    reason = "No confident periodic lock; vertical fit only"
            else:
                status = "no_signal"
                reason = "No fresh ADC samples"

        result = {
            "request_id": int(cfg["request_id"]),
            "active": not complete,
            "status": status,
            "complete": complete,
            "reason": reason,
            "fresh_samples": fresh_samples,
            "target_samples": int(cfg["min_samples"]),
            "max_wait_ms": float(cfg["max_wait_ms"]),
            "elapsed_ms": elapsed_ms,
            "seed_frequency_hz": (
                stats.get("frequency_hz") if stats else cfg.get("seed_frequency_hz")
            ),
            "stats": stats,
            "recommendation": recommendation,
        }

        with self._lock:
            if int(self._autoset["request_id"]) != int(cfg["request_id"]):
                return
            if complete:
                self._autoset["active"] = False
                self._autoset["status"] = status
            self._result["autoset"] = result

    def snapshot(self, compact=False):
        with self._lock:
            # JSON round-trip gives the caller an isolated, JSON-safe copy.
            import json
            result = json.loads(json.dumps(self._result))

        if compact:
            fft = result.get("fft", {})
            if "magnitude_dbv" in fft:
                fft["magnitude_dbv"] = []
                fft["payload_omitted"] = True

        return result

    def _stabilize_relation(self, comparison, source_generation):
        # v19.20: the comparison is intentionally stateless.  The previous
        # acquire/release lock was useful when a lag-search tried to decide
        # whether CH1 and CH2 represented the same signal.  The new loopback-
        # oriented LIVE comparison simply reports the current one-pass result.
        out = dict(comparison)
        raw_valid = bool(comparison.get("relation_valid", False))
        out["relation_raw_valid"] = raw_valid
        out["relation_valid"] = raw_valid
        out["valid"] = raw_valid
        out["relation_acquire_count"] = 1 if raw_valid else 0
        out["relation_release_count"] = 0 if raw_valid else 1
        return out

    def refresh_source(self, source, acquisition):
        requested = _requested_source_payload(source)

        with self._lock:
            self._result["requested"] = requested
            self._result["acquisition"].update(acquisition)
            self._result["updated_unix_s"] = time.time()


    def update_measurement(self, snapshot, source, acquisition):
        raw = snapshot.get("adc", [])
        counters = snapshot.get("counter", [])
        n = min(len(raw), MEASUREMENT_MAX_N)

        continuity = _continuity_from_counters(
            counters[-n:] if n > 0 else [],
            acquisition["fs_hz"],
        )

        quality = None

        if n <= 0:
            measurement = {
                "ready": False,
                "n": 0,
                "latest_adc_code": None,
                "latest_v": None,
                "mean_v": None,
                "min_v": None,
                "max_v": None,
                "vpp_v": None,
                "rms_v": None,
                "frequency_hz": None,
                "frequency_method": None,
                "crossing_frequency_hz": None,
                "period_s": None,
                "crossing_periods": 0,
                "periodic": False,
                "signal_class": "not_ready",
                "period_cv": None,
                "spectral_peak_hz": None,
                "spectral_prominence_db": None,
            }
        else:
            raw_window = raw[-n:]
            if _np is not None:
                raw_np = _np.asarray(raw_window, dtype=_np.float64)
                signal_np = raw_np * (ADC_VREF / ADC_FULL_SCALE)

                min_v = float(_np.min(signal_np))
                max_v = float(_np.max(signal_np))
                mean_v = float(_np.mean(signal_np))
                rms_v = float(_np.sqrt(_np.mean(signal_np * signal_np)))
                latest_v = float(signal_np[-1])
                quality = _best_effort_measurement_metrics_numpy(
                    signal_np, acquisition["fs_hz"]
                )
            else:
                # Preserve the standard-library fallback for environments
                # without NumPy.  UNO Q production currently uses NumPy.
                signal_v = [_raw_to_v(v) for v in raw_window]
                min_v = min(signal_v)
                max_v = max(signal_v)
                mean_v = sum(signal_v) / n
                rms_v = math.sqrt(sum(v * v for v in signal_v) / n)
                latest_v = signal_v[-1]
                quality = _best_effort_measurement_metrics(
                    signal_v, acquisition["fs_hz"]
                )

            freq_hz = quality["frequency_hz"]

            measurement = {
                "ready": True,
                "n": n,
                "latest_adc_code": int(raw_window[-1]),
                "latest_v": latest_v,
                "mean_v": mean_v,
                "min_v": min_v,
                "max_v": max_v,
                "vpp_v": max_v - min_v,
                "rms_v": rms_v,
                "frequency_hz": freq_hz,
                "frequency_method": quality["frequency_method"],
                "crossing_frequency_hz": quality["crossing_frequency_hz"],
                "period_s": (1.0 / freq_hz) if freq_hz else None,
                "crossing_periods": quality["crossing_periods"],
                "periodic": quality["periodic"],
                "signal_class": quality["classification"],
                "period_cv": quality["period_cv"],
                "spectral_peak_hz": quality["spectral_peak_hz"],
                "spectral_prominence_db": quality["spectral_prominence_db"],
            }

        compare_started = time.perf_counter()
        time_comparison_raw = _time_comparison(
            snapshot,
            source,
            acquisition,
            measurement_periodicity=quality,
        )
        compare_ms = (time.perf_counter() - compare_started) * 1000.0
        time_comparison = self._stabilize_relation(
            time_comparison_raw,
            int(source.get("generation", 0)),
        )

        with self._lock:
            trigger_config = dict(self._trigger_config)

        trigger = _trigger_analysis(
            snapshot,
            acquisition,
            trigger_config,
        )

        self._update_autoset(
            snapshot,
            source,
            acquisition,
        )

        with self._lock:
            self._result["measurement"] = measurement
            self._result["time_comparison"] = time_comparison
            self._result["trigger"] = trigger
            self._result["acquisition"].update(acquisition)
            self._result["acquisition"]["continuity"] = continuity
            self._result["analysis_ready"] = bool(
                measurement["ready"] and continuity["analysis_valid"]
            )
            self._result["updated_unix_s"] = time.time()

        return {"compare_ms": compare_ms}

    def update_fft(self, snapshot, source, acquisition):
        raw = snapshot.get("adc", [])
        counters = snapshot.get("counter", [])
        origins = snapshot.get("origin", [])

        # v19.22: the production FFT backend is NumPy, so keep the ADC window
        # in a NumPy array instead of converting 8192 samples through a Python
        # loop.  The scalar list path remains for no-NumPy fallback systems.
        if _np is not None:
            signal_v = (
                _np.asarray(raw, dtype=_np.float64)
                * (ADC_VREF / ADC_FULL_SCALE)
            )
        else:
            signal_v = [_raw_to_v(v) for v in raw]

        fft = _hann_fft(signal_v, acquisition["fs_hz"])

        fft_n = int(fft.get("n", 0) or 0)
        fft_continuity = _continuity_from_counters(
            counters[-fft_n:] if fft_n > 0 else [],
            acquisition["fs_hz"],
        )
        fft["analysis_valid"] = bool(
            fft.get("ready") and fft_continuity["analysis_valid"]
        )
        fft["continuity"] = fft_continuity

        transfer = {
            "valid": False,
            "reason": "ADC FFT not ready",
            "gain_db": None,
            "phase_deg": None,
            "thd_percent": None,
        }

        with self._lock:
            relation_locked = bool(
                self._result
                .get("time_comparison", {})
                .get("relation_valid", False)
            )

        if fft["ready"]:
            if not fft_continuity["analysis_valid"]:
                transfer["reason"] = fft_continuity["reason"]
            elif not source.get("running", False):
                transfer["reason"] = "DAC not running"
            elif not relation_locked:
                transfer["reason"] = "CH1/CH2 relation not locked"
            elif source.get("waveform") == "dc":
                transfer["reason"] = "DC has no transfer phase"
            elif float(source.get("frequency_hz", 0.0)) >= acquisition["nyquist_hz"]:
                transfer["reason"] = "Requested frequency is at/above ADC Nyquist"
            elif not counters or not origins:
                transfer["reason"] = "Reference timing unavailable"
            elif counters[0] < int(source.get("changed_counter", 0)):
                transfer["reason"] = "FFT window still contains pre-change source data"
            else:
                n = fft["n"]
                adc = signal_v[-n:]
                ctr = counters[-n:]
                org = origins[-n:]
                fs_hz = acquisition["fs_hz"]

                # CH1 is a virtual/known DAC reference, not a measured signal.
                # Evaluate that known waveform in one vector operation only to
                # retain exact sample/phase alignment with CH2; no CH1 FFT is
                # performed.  CH2 exact-tone/THD extraction is also vectorized.
                reference = _reference_values(ctr, org, source, fs_hz)

                f0 = float(source.get("frequency_hz", 0.0))
                adc_freqs = [f0]
                if source.get("waveform") == "sine":
                    for h in range(2, 6):
                        hf = f0 * h
                        if hf >= acquisition["nyquist_hz"]:
                            break
                        adc_freqs.append(hf)

                tone_ref = _exact_tones(reference, [f0], fs_hz).get(f0)
                adc_tones = _exact_tones(adc, adc_freqs, fs_hz)
                tone_adc = adc_tones.get(f0)

                if tone_ref and tone_adc and tone_ref["amp_vpk"] > 1e-9:
                    gain_db = 20.0 * math.log10(
                        max(tone_adc["amp_vpk"], 1e-12) /
                        tone_ref["amp_vpk"]
                    )

                    phase_deg = (
                        tone_adc["phase_rad"] - tone_ref["phase_rad"]
                    ) * 180.0 / math.pi
                    phase_deg = _wrap_phase_deg(phase_deg)

                    thd = None
                    if source.get("waveform") == "sine" and tone_adc["amp_vpk"] > 1e-9:
                        harmonic_power = 0.0
                        for hf in adc_freqs[1:]:
                            tone_h = adc_tones.get(hf)
                            if tone_h:
                                harmonic_power += tone_h["amp_vpk"] ** 2

                        thd = 100.0 * math.sqrt(harmonic_power) / tone_adc["amp_vpk"]

                    transfer = {
                        "valid": True,
                        "reason": "",
                        "gain_db": gain_db,
                        "phase_deg": phase_deg,
                        "thd_percent": thd,
                    }
                else:
                    transfer["reason"] = "Fundamental tone extraction failed"

        requested = _requested_source_payload(source)

        with self._lock:
            self._result["fft"] = fft
            self._result["transfer"] = transfer
            self._result["requested"] = requested
            self._result["acquisition"].update(acquisition)
            self._result["analysis_ready"] = bool(
                (
                    self._result["measurement"].get("ready")
                    and self._result["acquisition"]
                        .get("continuity", {})
                        .get("analysis_valid", False)
                )
                or fft.get("analysis_valid", False)
            )
            self._result["updated_unix_s"] = time.time()
