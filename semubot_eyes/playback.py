"""Read local SoX playback and its WAV envelope without opening audio devices."""

import os
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile


def wav_envelope(path, frame_seconds=0.04):
    """Return normalized RMS per frame; silence stays closed, stereo does not cancel."""
    rate, samples = wavfile.read(path)
    if samples.size == 0:
        return np.zeros(1), frame_seconds
    values = samples.astype(np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError('Non-finite WAV samples')
    if samples.dtype == np.uint8:
        values -= 128.0
    if values.ndim > 1:
        energy = np.mean(values * values, axis=1)
    else:
        energy = values * values
    frame_size = max(1, round(rate * frame_seconds))
    padding = (-len(energy)) % frame_size
    energy = np.pad(energy, (0, padding))
    rms = np.sqrt(energy.reshape(-1, frame_size).mean(axis=1))
    peak = float(rms.max())
    return rms / peak if peak > 0 else rms, frame_size / rate


class PlaybackTracker:
    """Poll only real play processes belonging to this user, at ten Hz."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.next_poll = 0.0
        self.key = None
        self.started = 0.0
        self.envelope = np.zeros(1)
        self.frame_seconds = 0.04
        self.ticks = os.sysconf('SC_CLK_TCK')

    def _find_playback(self):
        candidates = []
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                if process.stat().st_uid != os.getuid():
                    continue
                args = (process / 'cmdline').read_bytes().split(b'\0')
                if not args or Path(os.fsdecode(args[0])).name != 'play':
                    continue
                paths = [os.fsdecode(arg) for arg in args[1:]
                         if arg.lower().endswith(b'.wav')]
                if not paths:
                    continue
                # /proc stat fields after the parenthesized name start at field 3.
                fields = (process / 'stat').read_text().rsplit(')', 1)[1].split()
                start_ticks = int(fields[19])
                path = Path(paths[-1])
                if not path.is_absolute():
                    path = (process / 'cwd').resolve() / path
                candidates.append((start_ticks, int(process.name), str(path)))
            except (OSError, ValueError, IndexError):
                continue  # A process may finish between these reads.
        return max(candidates) if candidates else None

    def update(self):
        now = self.clock()
        if now < self.next_poll:
            return self.key is not None
        self.next_poll = now + 0.1
        key = self._find_playback()
        if key != self.key:
            self.key = key
            if key is not None:
                uptime = float(Path('/proc/uptime').read_text().split()[0])
                self.started = now - max(0.0, uptime - key[0] / self.ticks)
                try:
                    self.envelope, self.frame_seconds = wav_envelope(key[2])
                except (OSError, ValueError):
                    self.envelope = np.zeros(1)
        return self.key is not None

    def mouth_frame(self):
        if self.key is None:
            return 0
        index = int(max(0.0, self.clock() - self.started) / self.frame_seconds)
        if index >= len(self.envelope):
            return 0
        level = self.envelope[index]
        if level < 0.08:
            return 0
        if level < 0.3:
            return 1
        if level < 0.65:
            return 2
        return 3
