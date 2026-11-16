"""Pure proximity state: metres, hysteresis and explicit missing/stale data."""
import math


class Proximity:
    def __init__(self, threshold=.1, release=.12, timeout=2.):
        if not 0 < threshold <= release or timeout <= 0:
            raise ValueError('Require 0 < threshold <= release and timeout > 0')
        self.threshold, self.release, self.timeout = threshold, release, timeout
        self.channels = {i: dict(near=False, received=None, stamp=None, distance=None,
                                 kind='missing') for i in range(1, 5)}

    def update(self, channel, distance, minimum, maximum, now, stamp):
        state = self.channels[channel]
        state.update(received=now, stamp=stamp, distance=None)
        if (not math.isfinite(minimum) or not math.isfinite(maximum)
                or minimum < 0 or maximum <= minimum or math.isnan(distance)):
            state['kind'] = 'invalid'
            return
        if distance == -math.inf or distance < minimum:
            state.update(kind='too_close', near=True)
        elif distance == math.inf or distance > maximum:
            state.update(kind='no_echo', near=False)
        else:
            state.update(kind='range', distance=distance)
            # Float32 Range values can represent exactly 0.1 m just above 0.1.
            if distance <= self.threshold + 1e-7:
                state['near'] = True
            elif distance > self.release + 1e-7:
                state['near'] = False

    def snapshot(self, now, sim_time):
        channels = []
        for channel, value in self.channels.items():
            item = dict(value, channel=channel)
            item['age'] = None if item['received'] is None else max(0., now-item['received'])
            age_sim = None if item['stamp'] is None else sim_time-item['stamp']
            if item['age'] is not None and (item['age'] > self.timeout or
                    age_sim > self.timeout or age_sim < -.1):
                item['kind'] = 'stale'
            item.pop('received'); item.pop('stamp')
            channels.append(item)
        return dict(threshold=self.threshold, release=self.release,
                    alarm=any(c['near'] for c in channels),
                    unknown=any(c['kind'] in ('missing', 'invalid', 'stale') for c in channels),
                    channels=channels)
