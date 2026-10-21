"""Activity selection independent of ROS and rendering."""

import json
import time


STAGE_EXPRESSIONS = {
    'manual': 'idle', 'idle': 'idle',
    'listening': 'listening', 'recording': 'listening',
    'transcribing': 'thinking', 'thinking': 'thinking',
    'synthesizing': 'thinking', 'speaking': 'speaking',
    'error': 'error', 'disconnected': 'error',
}


class ExpressionState:
    """Use DDS publisher presence, not stage age: idle stages can last hours."""

    def __init__(self, clock=time.monotonic, wall_clock=time.time,
                 disconnect_grace=3.0, error_seconds=5.0):
        self.clock = clock
        self.wall_clock = wall_clock
        self.disconnect_grace = disconnect_grace
        self.error_seconds = error_seconds
        self.stage = None
        self.missing_since = clock()
        self.error_until = 0.0
        self.seen_errors = set()

    def receive_state(self, payload):
        try:
            state = json.loads(payload)
        except (ValueError, TypeError):
            return False
        if not isinstance(state, dict) or not isinstance(state.get('stage'), str) \
                or state['stage'] not in STAGE_EXPRESSIONS:
            return False
        self.stage = state['stage']
        return True

    def receive_log(self, payload):
        try:
            entries = json.loads(payload)
        except (ValueError, TypeError):
            return False
        if not isinstance(entries, list):
            return False
        keys = set()
        for entry in entries:
            if not isinstance(entry, dict) or entry.get('kind') != 'error':
                continue
            stamp = entry.get('time')
            if not isinstance(stamp, (int, float)):
                continue
            key = (stamp, str(entry.get('text', '')))
            keys.add(key)
            age = self.wall_clock() - stamp
            if key not in self.seen_errors and 0 <= age < self.error_seconds:
                self.error_until = max(self.error_until,
                                       self.clock() + self.error_seconds - age)
        self.seen_errors = keys
        return True

    def expression(self, publisher_present, audio_playing=False):
        now = self.clock()
        if publisher_present and self.stage is not None:
            self.missing_since = None
        elif self.missing_since is None:
            self.missing_since = now
        # Local sayhi / speak -a clips still animate without the brain.
        if audio_playing:
            return 'speaking'
        if self.missing_since is not None and now - self.missing_since >= self.disconnect_grace:
            return 'error'
        if now < self.error_until:
            return 'error'
        return STAGE_EXPRESSIONS.get(self.stage, 'idle')
