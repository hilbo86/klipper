# Motion / idle force comparison for filament handling
#
# Copyright (C) 2026 Timo Hilbig <gh@t-hilbig.de>
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections
import statistics


class MotionForceSampler:
    def __init__(self, config):
        self.reactor = config.get_printer().get_reactor()
        self.minimum_delta = config.getfloat(
            "motion_force_delta", 25.0, above=0.0)
        self.sample_time = config.getfloat(
            "motion_sample_time", 0.3, above=0.0)
        self.settle_time = config.getfloat(
            "motion_settle_time", 0.2, minval=0.0)
        self.confirmations = config.getint(
            "motion_confirmations", 2, minval=2)
        self.timeout = config.getfloat(
            "motion_sample_timeout", 2.0, above=0.0)
        self.samples = collections.deque(maxlen=4096)
        self.matches = 0
        self.delta = 0.0

    def reset(self):
        self.samples.clear()
        self.matches = 0
        self.delta = 0.0

    def add_sample(self, sample):
        self.samples.append(
            (sample["print_time"], sample["absolute_force_g"]))

    def collect(self, gcmd, start, end):
        # Wait for delivery of the entire interval, including delayed ADC
        # callbacks. Classify by MCU print_time, never by callback arrival.
        deadline = self.reactor.monotonic() + self.timeout
        while not self.samples or self.samples[-1][0] < end:
            if self.reactor.monotonic() >= deadline:
                raise gcmd.error("Timeout collecting motion force samples")
            self.reactor.pause(self.reactor.monotonic() + 0.02)
        return [force for time, force in self.samples if start <= time < end]

    def idle(self, gcmd, tool):
        start = tool.get_last_move_time() + self.settle_time
        tool.dwell(self.settle_time + self.sample_time)
        tool.wait_moves()
        return self.collect(gcmd, start, start + self.sample_time)

    def check(self, moving, before, after, baseline, direction, force_limit):
        self.delta = 0.0
        if min(len(moving), len(before), len(after)) < 3:
            self.matches = 0
            return False
        moving_force = direction * (statistics.median(moving) - baseline)
        # Comparing both sides rejects a static elastic force increase and
        # slow baseline drift. Unload uses the opposite force polarity.
        idle_force = max(direction * (statistics.mean(values) - baseline)
                         for values in (before, after))
        self.delta = moving_force - idle_force
        noise = max(statistics.pstdev(values) for values in (before, after))
        matched = (0.0 < moving_force < force_limit
                   and all(abs(value - baseline) < force_limit
                           for values in (moving, before, after)
                           for value in values)
                   and self.delta >= max(self.minimum_delta, 3.0 * noise))
        self.matches = self.matches + 1 if matched else 0
        return self.matches >= self.confirmations
