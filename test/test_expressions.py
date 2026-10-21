import json
import tempfile
import subprocess
import sys
import time
import unittest
from pathlib import Path

import numpy as np
from scipy.io import wavfile

from semubot_eyes.expressions import ExpressionState
from semubot_eyes.playback import PlaybackTracker, wav_envelope


class ExpressionTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.state = ExpressionState(clock=lambda: self.now,
                                     wall_clock=lambda: 1000 + self.now)

    def test_all_brain_stages(self):
        for stage, expected in [('manual', 'idle'), ('listening', 'listening'),
                                ('recording', 'listening'), ('thinking', 'thinking'),
                                ('transcribing', 'thinking'), ('synthesizing', 'thinking'),
                                ('speaking', 'speaking'), ('error', 'error')]:
            self.assertTrue(self.state.receive_state(json.dumps({'stage': stage})))
            self.assertEqual(self.state.expression(True), expected)

    def test_disconnect_grace_recovery_and_long_idle(self):
        self.state.receive_state('{"stage":"manual"}')
        self.assertEqual(self.state.expression(True), 'idle')
        self.now = 3600
        self.assertEqual(self.state.expression(True), 'idle')
        self.assertEqual(self.state.expression(False), 'idle')
        self.now += 3.1
        self.assertEqual(self.state.expression(False), 'error')
        self.state.receive_state('{"stage":"listening"}')
        self.assertEqual(self.state.expression(True), 'listening')

    def test_startup_no_state_and_local_audio(self):
        self.assertEqual(self.state.expression(False), 'idle')
        self.now = 4
        self.assertEqual(self.state.expression(False), 'error')
        self.assertEqual(self.state.expression(False, True), 'speaking')
        self.assertEqual(self.state.expression(True), 'error')

    def test_reject_bad_messages_without_losing_activity(self):
        self.state.receive_state('{"stage":"listening"}')
        for payload in ('broken', '[]', '{}', '{"stage":[]}', '{"stage":"unknown"}'):
            self.assertFalse(self.state.receive_state(payload))
        self.assertEqual(self.state.expression(True), 'listening')

    def test_recent_error_expires_and_is_not_replayed(self):
        self.state.receive_state('{"stage":"listening"}')
        stale = json.dumps([{'kind': 'error', 'time': 900, 'text': 'old'}])
        self.state.receive_log(stale)
        self.assertEqual(self.state.expression(True), 'listening')
        recent = json.dumps([{'kind': 'error', 'time': 1000, 'text': 'new'}])
        self.state.receive_log(recent)
        self.assertEqual(self.state.expression(True), 'error')
        self.now = 4
        self.state.receive_log(recent)
        self.now = 5.1
        self.assertEqual(self.state.expression(True), 'listening')


class PlaybackTests(unittest.TestCase):
    def test_detect_real_process_and_stop_without_playing_sound(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sample.wav'
            wavfile.write(path, 16000, np.zeros(1600, dtype=np.int16))
            # Emulate SoX's command line in /proc without opening any audio device.
            process = subprocess.Popen(['play', '-c', 'import time; time.sleep(5)', str(path)],
                                       executable=sys.executable)
            try:
                tracker = PlaybackTracker()
                deadline = time.monotonic() + 2
                while tracker.key is None and time.monotonic() < deadline:
                    tracker.update()
                    time.sleep(0.02)
                self.assertIsNotNone(tracker.key)
                self.assertEqual(tracker.key[1], process.pid)
                self.assertEqual(tracker.mouth_frame(), 0)
            finally:
                process.terminate()
                process.wait(timeout=3)
            tracker.next_poll = 0
            self.assertFalse(tracker.update())

    def test_silence_speech_pause_stereo_and_unsigned_pcm(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sample.wav'
            rate = 16000
            tone = (20000 * np.sin(np.arange(640) * 2 * np.pi * 440 / rate)).astype(np.int16)
            samples = np.concatenate((np.zeros(640, dtype=np.int16), tone,
                                      np.zeros(640, dtype=np.int16)))
            for audio in (samples, np.column_stack((samples, -samples))):
                wavfile.write(path, rate, audio)
                envelope, duration = wav_envelope(path)
                self.assertAlmostEqual(duration, 0.04)
                np.testing.assert_allclose(envelope, [0, 1, 0])
            wavfile.write(path, rate, np.full(640, 128, dtype=np.uint8))
            envelope, _ = wav_envelope(path)
            self.assertEqual(envelope[0], 0)

    def test_mouth_tracks_audio_and_closes_at_end(self):
        now = [10.0]
        tracker = PlaybackTracker(clock=lambda: now[0])
        tracker.key = (0, 123, 'speech.wav')
        tracker.started = 10.0
        tracker.frame_seconds = 0.04
        tracker.envelope = np.array([0, 0.2, 0.5, 1, 0])
        for i, expected in enumerate((0, 1, 2, 3, 0, 0)):
            now[0] = 10 + i * 0.04 + 0.001
            self.assertEqual(tracker.mouth_frame(), expected)
        tracker.key = None
        self.assertEqual(tracker.mouth_frame(), 0)


if __name__ == '__main__':
    unittest.main()
