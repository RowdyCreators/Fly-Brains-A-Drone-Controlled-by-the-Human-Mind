"""Live bridge: feed signals data, live, into processing.

WARNING -- DO NOT DRIVE A REAL DRONE WITH THIS YET. This bridge
only connects the pipeline to the processor; it does not prove the
signal is a real command. Before any real flight: record a labelled
per-user training session and load its profile (see
``processing/calibrate.py`` and ``SignalProcessor.from_profile``),
bench-test the whole chain (Pi + DAC + remote, props off) with the
real pipeline, and keep a physical kill switch / a way to force
hover from the remote. The thresholds are still starting points,
the simulator's signal drifts, and until a labelled session proves
otherwise the axes can move on drift alone. Dry-run only until then.

This is the missing link for Fly-Brains issue #14 ("Create pipeline
between signals and processing"): it takes the data the ``signals/``
side produces, as it arrives, and hands it to ``SignalProcessor``
(``processing/processor.py``), one sample at a time, yielding each
processed result as soon as it exists.

Two kinds of signals data, two entry points:

1. Raw triples -- what ``signals/signal.py`` produces. ``FlyBrain``
   gives ``(timestamp, alpha, beta, gamma)`` in volts (or the
   simulator gives ``(alpha, beta, gamma)``). Feed those straight
   in with :func:`stream_to_processor` or :class:`PipelineBridge.on_sample`.
   (``processing/main.py`` already does this in its live mode, via
   ``FlyBrain.listen(on_sample=...)``; this module is the same idea
   as a reusable function, so any source -- a file, a socket, a
   generator -- can drive the processor.)

2. Band powers -- what ``signals/eeg_pipeline.py`` (on ``main``)
   produces. Every 0.25 s it computes ``band_powers()`` and returns
   ``(abs_p, rel_p)``, each shaped ``(channels, 3)`` with columns
   alpha / beta / gamma, one row per electrode. Its own
   ``decide_command()`` is a stub returning ``None`` -- this module
   is the replacement for that stub: :func:`stream_band_powers_to_processor`
   or :class:`PipelineBridge.on_band_powers` turn each band-power
   update into a processor result (state, command, active channel,
   and, with a mapper, drone axes).

   Which band power to feed: ``rel_p`` (relative power, the fraction
   of 1-45 Hz power in each band), not ``abs_p``. The simulation in
   ``sim_real_life.py`` showed why: absolute band powers are ~1e-5 V^2,
   far below the processor's ``MIN_STD`` floor of 0.01, so every
   z-score is ~0.001 and the drone never moves. Relative powers are
   fractions around 0.02-0.19, so their variation is well above the
   floor and the processor can actually see changes.

   Which electrode: ``electrode=0`` by default, i.e. the first row of
   ``rel_p``. The pipeline has 3 electrodes and 3 bands, the processor
   has 3 channels -- this bridge reads the 3 bands of ONE electrode
   as the processor's alpha / beta / gamma. That is a choice the team
   still has to confirm (see ``sim_real_life_report.md``: the repo
   currently disagrees about what a "channel" is). If the team decides
   a channel is a band across electrodes instead, only
   ``band_powers_to_sample`` needs to change.

Stdlib only in this module itself (no numpy import): band-power rows
are read by indexing, so plain lists and numpy arrays both work.
:func:`iter_eeg_pipeline` does need numpy/scipy, because it reuses
``signals/eeg_pipeline``'s own ``Acquirer`` / ``StreamCleaner`` /
``band_powers`` -- it is skipped with a clear error when they are
not importable.

Usage -- raw triples, live:

    from processing.live_bridge import stream_to_processor
    from signals.brain_sim import simulate_brain

    for result in stream_to_processor(simulate_brain(rate_hz=10, max_samples=30)):
        print(result["state"], result["command"], result["active_channel"])

Usage -- band powers from the real pipeline (drop-in for its
``decide_command`` stub; put this in ``eeg_pipeline.py``'s main loop):

    from processing.live_bridge import PipelineBridge
    bridge = PipelineBridge(mapper=DroneMapper())
    ...
    abs_p, rel_p = band_powers(np.array(window).T, FS)
    result = bridge.on_band_powers(abs_p, rel_p)
    if result is not None:
        drone.set(**result["axes"])   # or result["command"]

Usage -- whole pipeline, simulated, from the command line:

    python -m processing.live_bridge --sim --seconds 10
"""

import time

from processing.drone_map import DroneMapper, axes_to_command
from processing.processor import CHANNELS, SignalProcessor, drive_drone

# Which electrode's band powers feed the processor. The pipeline
# returns one row per electrode (3 EXG pills); the processor has 3
# channels, so this bridge reads the 3 bands of ONE electrode as
# alpha / beta / gamma. Default 0 = the first pill, wired to ADS1115
# A0 (see "Team decisions" in drone-mapping-plan.md -- this is a
# PROPOSED decision, pending team sign-off; change it with the
# electrode= argument, not by editing this constant in place).
DEFAULT_ELECTRODE = 0


def band_powers_to_sample(rel_p, electrode=0, timestamp=None):
    """Turn one band-power update into a processor sample.

    ``rel_p`` is one row per electrode, columns alpha / beta / gamma
    (relative power, 0..1). Returns ``(timestamp, alpha, beta, gamma)``
    -- the same shape ``SignalProcessor.add_sample`` accepts.
    ``abs_p`` is deliberately not used; see the module docstring.
    """
    row = rel_p[electrode]
    values = [float(row[i]) for i in range(3)]
    return (timestamp, values[0], values[1], values[2])


def _send_result(result, mapper, drone, on_result):
    """Attach axes / drive a drone / call back for one result."""
    if mapper is not None:
        if drone is not None:
            result["axes"] = mapper.drive(drone, result)
        else:
            result["axes"] = mapper.update(result)
        # Command and axes must come from the same decision (PR #15):
        # the command is the axes' summary, not the processor state's,
        # so the output can never again show command=hover with the
        # axes at +/-0.50.
        result["command"] = axes_to_command(result["axes"])
    elif drone is not None:
        drive_drone(drone, result)
    if on_result is not None:
        on_result(result)
    return result


def stream_to_processor(source, processor=None, mapper=None,
                        drone=None, on_result=None):
    """Feed a live stream of samples into the processor, as they arrive.

    ``source`` is any iterable of samples: ``(timestamp, alpha, beta,
    gamma)`` tuples, ``(alpha, beta, gamma)`` tuples, or dicts with
    ``alpha`` / ``beta`` / ``gamma`` keys -- whatever
    ``SignalProcessor.add_sample`` accepts. Yields each processed
    result dict as soon as the processor produces one (nothing is
    yielded during the first ``calibrate_samples`` samples, while the
    baseline is being collected, or for dropped samples).

    With ``mapper`` given, each result also gets ``result["axes"]``
    (Option C: alpha->pitch, beta->roll, gamma->throttle, yaw 0).
    With ``drone`` given too, the axes are sent via ``drone.set()``
    on every window (feeding the Drone's 0.5 s watchdog); with a
    drone but no mapper, the command is sent via ``drone.command()``.
    ``on_result`` is called with each result, if given.
    """
    if processor is None:
        processor = SignalProcessor()
    for sample in source:
        result = processor.add_sample(sample)
        if result is None:
            continue
        yield _send_result(result, mapper, drone, on_result)


def stream_band_powers_to_processor(band_power_stream, processor=None,
                                    electrode=DEFAULT_ELECTRODE, mapper=None, drone=None,
                                    on_result=None):
    """Feed a live stream of band-power updates into the processor.

    ``band_power_stream`` yields ``(abs_p, rel_p)`` pairs (as from
    ``signals.eeg_pipeline.band_powers``) or ``rel_p`` alone. Each
    update becomes one sample via :func:`band_powers_to_sample`
    (``rel_p`` row ``electrode``) and is fed to the processor, so
    one pipeline update (every 0.25 s) gives one processed result.
    """
    def _samples():
        for item in band_power_stream:
            if isinstance(item, tuple) and len(item) == 2:
                _, rel_p = item
            else:
                rel_p = item
            yield band_powers_to_sample(rel_p, electrode=electrode,
                                        timestamp=time.time())

    yield from stream_to_processor(
        _samples(), processor=processor, mapper=mapper,
        drone=drone, on_result=on_result)


class PipelineBridge:
    """One object holding the whole signals -> processing -> drone chain.

    The drop-in for ``eeg_pipeline.decide_command``: create it once,
    then call :meth:`on_band_powers` on every band-power update (or
    :meth:`on_sample` on every raw triple). It returns the processor
    result dict for that update, or ``None`` while the processor is
    still calibrating its baseline.
    """

    def __init__(self, processor=None, mapper=None, electrode=DEFAULT_ELECTRODE):
        self.processor = processor if processor is not None else SignalProcessor()
        self.mapper = mapper
        self.electrode = electrode

    def on_sample(self, sample, drone=None, on_result=None):
        """Feed one raw (timestamp, alpha, beta, gamma) sample."""
        result = self.processor.add_sample(sample)
        if result is None:
            return None
        return _send_result(result, self.mapper, drone, on_result)

    def on_band_powers(self, abs_p, rel_p, timestamp=None, drone=None,
                       on_result=None):
        """Feed one ``band_powers()`` update. ``abs_p`` is accepted for
        signature compatibility with ``decide_command`` and ignored."""
        sample = band_powers_to_sample(rel_p, electrode=self.electrode,
                                       timestamp=timestamp)
        return self.on_sample(sample, drone=drone, on_result=on_result)


def iter_eeg_pipeline(source, fs=125, window_sec=2.0, update_sec=0.25,
                      max_updates=None):
    """Run main's real EEG pipeline and yield ``(abs_p, rel_p)`` live.

    Reuses ``signals.eeg_pipeline``'s own ``Acquirer`` (background
    sampling thread), ``StreamCleaner`` (DC + 60 Hz notch) and
    ``band_powers`` (Welch, 2 s window every 0.25 s). ``source`` is an
    ``ADSSource`` or ``SimSource`` from that module. Yields nothing
    while the 2 s window is still filling. Needs numpy/scipy; raises
    ImportError with a clear message if they (or the pipeline) are
    not importable.
    """
    try:
        from signals import eeg_pipeline
    except ImportError as exc:
        raise ImportError(
            "iter_eeg_pipeline needs signals/eeg_pipeline.py from main "
            "(and numpy/scipy): %s" % exc) from exc

    from collections import deque

    cleaner = eeg_pipeline.StreamCleaner(fs, eeg_pipeline.CHANNELS)
    win_len = int(window_sec * fs)
    window = deque(maxlen=win_len)
    acq = eeg_pipeline.Acquirer(source, fs)
    acq.start()
    try:
        count = 0
        while max_updates is None or count < max_updates:
            time.sleep(update_sec)
            items = acq.drain()
            if len(items) < 2:
                continue
            import numpy as np
            raw = np.array([v for _, v in items]).T
            clean = cleaner.process(raw)
            window.extend(clean.T)
            if len(window) < win_len:
                continue
            yield eeg_pipeline.band_powers(np.array(window).T, fs)
            count += 1
    finally:
        acq.running = False


def stream_eeg_pipeline(source, processor=None, electrode=0, mapper=None,
                        drone=None, on_result=None, fs=125, window_sec=2.0,
                        update_sec=0.25, max_updates=None):
    """Whole chain, live: pipeline source -> band powers -> processor.

    One call runs ``iter_eeg_pipeline`` and feeds every update to the
    processor, yielding each result. ``source`` is an ``ADSSource`` or
    ``SimSource`` from ``signals.eeg_pipeline``.
    """
    yield from stream_band_powers_to_processor(
        iter_eeg_pipeline(source, fs=fs, window_sec=window_sec,
                          update_sec=update_sec, max_updates=max_updates),
        processor=processor, electrode=electrode, mapper=mapper,
        drone=drone, on_result=on_result)


def _demo(seconds=10):
    """Simulated live run: eeg_pipeline SimSource -> processor -> print."""
    from signals.eeg_pipeline import SimSource
    print("Simulated EEG pipeline -> processor, %ss (Ctrl+C to stop)..." % seconds)
    n_updates = max(1, int(seconds / 0.25))
    for result in stream_eeg_pipeline(
            SimSource(), mapper=DroneMapper(), max_updates=n_updates):
        axes = result.get("axes") or {}
        z = result["z"]
        print(f"t={result['timestamp']:.1f}  "
              f"smoothed=({result['smoothed'][0]:.3f},{result['smoothed'][1]:.3f},"
              f"{result['smoothed'][2]:.3f})  "
              f"z=({z['alpha']:+.2f},{z['beta']:+.2f},{z['gamma']:+.2f})  "
              f"state={result['state']} command={result['command']}  "
              f"pitch={axes.get('pitch', 0.0):+.2f} "
              f"roll={axes.get('roll', 0.0):+.2f} "
              f"throttle={axes.get('throttle', 0.0):+.2f}")
    print("done")


if __name__ == "__main__":
    import argparse

    _parser = argparse.ArgumentParser(description="Live signals -> processing demo")
    _parser.add_argument("--sim", action="store_true",
                         help="use the simulated EEG source")
    _parser.add_argument("--seconds", type=float, default=10)
    _args = _parser.parse_args()
    if not _args.sim:
        _parser.error("only --sim is supported off-Pi (no ADS1115 here)")
    _demo(seconds=_args.seconds)
