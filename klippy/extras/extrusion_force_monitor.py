# Time-synchronised extrusion force monitoring
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math


IDLE = "IDLE"
TRAVEL = "TRAVEL"
RETRACT = "RETRACT"
EXTRUSION_TRANSIENT = "EXTRUSION_TRANSIENT"
EXTRUSION_STEADY = "EXTRUSION_STEADY"


class ExponentialFilter:
    def __init__(self, tau):
        self.tau = tau
        self.value = None
        self.time = None

    def update(self, value, sample_time):
        if self.value is None or self.time is None or sample_time <= self.time:
            self.value = value
            self.time = sample_time
            return value
        dt = sample_time - self.time
        alpha = 1.0 - math.exp(-dt / self.tau)
        self.value += alpha * (value - self.value)
        self.time = sample_time
        return self.value


class NoiseEstimator:
    def __init__(self, tau):
        self.tau = tau
        self.mean = None
        self.variance = 0.0
        self.time = None

    def update(self, value, sample_time):
        if self.mean is None or self.time is None or sample_time <= self.time:
            self.mean = value
            self.time = sample_time
            return math.sqrt(self.variance)
        dt = sample_time - self.time
        alpha = 1.0 - math.exp(-dt / self.tau)
        delta = value - self.mean
        self.mean += alpha * delta
        self.variance = ((1.0 - alpha) * self.variance
                         + alpha * delta * (value - self.mean))
        self.time = sample_time
        return math.sqrt(max(0.0, self.variance))


class BaselineTracker:
    def __init__(self, tau, quiet_time, maximum_deviation, noise_factor):
        self.tau = tau
        self.quiet_time = quiet_time
        self.maximum_deviation = maximum_deviation
        self.noise_factor = noise_factor
        self.value = None
        self.time = None
        self.last_extrusion_time = None

    def update(self, force, sample_time, motion_state, noise):
        if motion_state in (EXTRUSION_TRANSIENT, EXTRUSION_STEADY, RETRACT):
            self.last_extrusion_time = sample_time
            return self.value
        if motion_state not in (IDLE, TRAVEL):
            return self.value
        if (self.last_extrusion_time is not None
                and sample_time - self.last_extrusion_time < self.quiet_time):
            return self.value
        if self.value is None:
            self.value = force
            self.time = sample_time
            return self.value
        allowed = max(self.maximum_deviation, noise * self.noise_factor)
        if abs(force - self.value) > allowed:
            return self.value
        if self.time is None or sample_time <= self.time:
            self.value = force
        else:
            dt = sample_time - self.time
            alpha = 1.0 - math.exp(-dt / self.tau)
            self.value += alpha * (force - self.value)
        self.time = sample_time
        return self.value


class MotionClassifier:
    def __init__(self, extrusion_epsilon, xy_epsilon, minimum_flow,
                 transient_time, flow_change_ratio):
        self.extrusion_epsilon = extrusion_epsilon
        self.xy_epsilon = xy_epsilon
        self.minimum_flow = minimum_flow
        self.transient_time = transient_time
        self.flow_change_ratio = flow_change_ratio
        self.last_flow = 0.0
        self.steady_since = None

    def classify(self, sample_time, e_velocity, xy_velocity, flow):
        if e_velocity < -self.extrusion_epsilon:
            self.last_flow = 0.0
            self.steady_since = None
            return RETRACT
        if e_velocity <= self.extrusion_epsilon:
            self.last_flow = 0.0
            self.steady_since = None
            return TRAVEL if xy_velocity > self.xy_epsilon else IDLE
        if flow < self.minimum_flow:
            self.last_flow = flow
            self.steady_since = None
            return EXTRUSION_TRANSIENT
        denominator = max(abs(self.last_flow), flow, self.minimum_flow)
        changed = (self.last_flow < self.minimum_flow
                   or abs(flow - self.last_flow) / denominator
                   > self.flow_change_ratio)
        if changed or self.steady_since is None:
            self.steady_since = sample_time
        self.last_flow = flow
        if sample_time - self.steady_since < self.transient_time:
            return EXTRUSION_TRANSIENT
        return EXTRUSION_STEADY


class ExtrusionForceProcessor:
    """Pure processing core used by live monitoring and offline replay."""
    def __init__(self, fast_filter_tau=0.08, control_filter_tau=0.25,
                 trend_filter_tau=2.0, baseline_update_tau=1.0,
                 baseline_quiet_time=0.5, baseline_max_deviation=250.0,
                 baseline_noise_factor=6.0, noise_tau=3.0,
                 extrusion_epsilon=0.0001, xy_epsilon=0.01,
                 minimum_flow=0.5, transient_time=0.30,
                 flow_change_ratio=0.10, max_confident_noise=100.0):
        self.fast_filter = ExponentialFilter(fast_filter_tau)
        self.control_filter = ExponentialFilter(control_filter_tau)
        self.trend_filter = ExponentialFilter(trend_filter_tau)
        self.noise = NoiseEstimator(noise_tau)
        self.baseline = BaselineTracker(
            baseline_update_tau, baseline_quiet_time,
            baseline_max_deviation, baseline_noise_factor)
        self.classifier = MotionClassifier(
            extrusion_epsilon, xy_epsilon, minimum_flow,
            transient_time, flow_change_ratio)
        self.max_confident_noise = max_confident_noise
        self.last_fast_force = None
        self.last_force_time = None
        self.dynamic_expected = {}
        self.dynamic_time = {}
        self.latest = None

    def _dynamic_force(self, extruder, expected, profile, sample_time):
        if expected is None or profile is None:
            self.dynamic_expected.pop(extruder, None)
            self.dynamic_time.pop(extruder, None)
            return None
        previous = self.dynamic_expected.get(extruder)
        previous_time = self.dynamic_time.get(extruder)
        if previous is None or previous_time is None or sample_time <= previous_time:
            result = expected
        else:
            tau = (profile.response_tau_rise if expected >= previous
                   else profile.response_tau_fall)
            alpha = 1.0 - math.exp(-(sample_time - previous_time) / tau)
            result = previous + alpha * (expected - previous)
        self.dynamic_expected[extruder] = result
        self.dynamic_time[extruder] = sample_time
        return result

    def process(self, observation, profile=None):
        sample_time = float(observation["print_time"])
        absolute_force = float(observation["absolute_force_g"])
        e_velocity = float(observation.get("e_velocity", 0.0))
        xy_velocity = float(observation.get("xy_velocity", 0.0))
        flow = max(0.0, float(observation.get("flow_mm3_s", 0.0)))
        extruder = observation.get("extruder")
        temperature = float(observation.get("temperature", 0.0))
        target_temperature = float(
            observation.get("target_temperature", temperature))
        motion_state = self.classifier.classify(
            sample_time, e_velocity, xy_velocity, flow)

        noise = math.sqrt(max(0.0, self.noise.variance))
        baseline = self.baseline.update(
            absolute_force, sample_time, motion_state, noise)
        force = (absolute_force - baseline
                 if baseline is not None else 0.0)
        if motion_state in (IDLE, TRAVEL):
            noise = self.noise.update(force, sample_time)
        force_fast = self.fast_filter.update(force, sample_time)
        force_control = self.control_filter.update(force, sample_time)
        force_trend = self.trend_filter.update(force, sample_time)
        dforce_dt = 0.0
        if (self.last_fast_force is not None and self.last_force_time is not None
                and sample_time > self.last_force_time):
            dforce_dt = ((force_fast - self.last_fast_force)
                         / (sample_time - self.last_force_time))
        self.last_fast_force = force_fast
        self.last_force_time = sample_time

        expected = None
        if profile is not None and motion_state in (
                EXTRUSION_TRANSIENT, EXTRUSION_STEADY):
            expected = profile.expected_force(flow, temperature)
        dynamic_expected = self._dynamic_force(
            extruder, expected, profile, sample_time)
        excess = (force_control - dynamic_expected
                  if dynamic_expected is not None else None)

        confidence = 1.0
        if baseline is None:
            confidence -= 0.4
        if profile is None or expected is None:
            confidence -= 0.4
        if motion_state == EXTRUSION_TRANSIENT:
            confidence -= 0.25
        elif motion_state == RETRACT:
            confidence -= 0.4
        elif motion_state in (IDLE, TRAVEL):
            confidence -= 0.2
        if abs(temperature - target_temperature) > 2.0:
            confidence -= 0.15
        if noise > self.max_confident_noise:
            confidence -= 0.2
        confidence = max(0.0, min(1.0, confidence))

        self.latest = {
            "print_time": sample_time,
            "extruder": extruder,
            "absolute_force_g": absolute_force,
            "baseline_force_g": baseline,
            "force_g": force,
            "force_fast_g": force_fast,
            "force_control_g": force_control,
            "force_trend_g": force_trend,
            "noise_g": noise,
            "dforce_dt": dforce_dt,
            "e_position": observation.get("e_position"),
            "e_velocity": e_velocity,
            "xy_velocity": xy_velocity,
            "flow_mm3_s": flow,
            "temperature": temperature,
            "target_temperature": target_temperature,
            "expected_force_g": expected,
            "expected_dynamic_force_g": dynamic_expected,
            "excess_force_g": excess,
            "motion_state": motion_state,
            "profile": profile.name if profile is not None else None,
            "confidence": confidence,
        }
        return dict(self.latest)


def replay_rows(rows, processor, profile_resolver=None):
    """Replay CSV-like dictionaries through the live processing core."""
    states = []
    for row in rows:
        extruder = row.get("extruder", "extruder")
        profile = (profile_resolver(extruder) if profile_resolver is not None
                   else None)
        observation = {
            "print_time": float(row["print_time"]),
            "absolute_force_g": float(
                row.get("absolute_force_g", row.get("force", 0.0))),
            "extruder": extruder,
            "e_position": (float(row["e_position"])
                           if row.get("e_position") not in (None, "")
                           else None),
            "e_velocity": float(row.get("e_velocity", 0.0)),
            "xy_velocity": float(row.get("xy_velocity", 0.0)),
            "flow_mm3_s": float(row.get("flow_mm3_s", row.get("flow", 0.0))),
            "temperature": float(row.get("temperature", 0.0)),
            "target_temperature": float(
                row.get("target_temperature", row.get("temperature", 0.0))),
        }
        states.append(processor.process(observation, profile))
    return states


class TrapQMotionProvider:
    def __init__(self, name, trapq, extruder=False):
        import chelper
        self.name = name
        self.trapq = trapq
        self.extruder = extruder
        self.ffi_main, self.ffi_lib = chelper.get_ffi()

    def get_state(self, print_time):
        data = self.ffi_main.new("struct pull_move[1]")
        count = self.ffi_lib.trapq_extract_old(
            self.trapq, data, 1, 0.0, print_time)
        if not count:
            return None
        move = data[0]
        move_time = max(0.0, min(move.move_t, print_time - move.print_time))
        distance = ((move.start_v + 0.5 * move.accel * move_time)
                    * move_time)
        velocity = move.start_v + move.accel * move_time
        if self.extruder:
            return {
                "extruder": self.name,
                "e_position": move.start_x + move.x_r * distance,
                "e_velocity": velocity * move.x_r,
            }
        xy_ratio = math.sqrt(move.x_r * move.x_r + move.y_r * move.y_r)
        return {"xy_velocity": abs(velocity) * xy_ratio}


class ExtrusionForceMonitor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()
        self.load_cell_name = config.get("load_cell", "load_cell")
        self.callback_interval = 1.0 / config.getfloat(
            "callback_rate", 20.0, above=0.0, maxval=20.0)
        self.processor = ExtrusionForceProcessor(
            fast_filter_tau=config.getfloat(
                "fast_filter_tau", 0.08, above=0.0),
            control_filter_tau=config.getfloat(
                "control_filter_tau", 0.25, above=0.0),
            trend_filter_tau=config.getfloat(
                "trend_filter_tau", 2.0, above=0.0),
            baseline_update_tau=config.getfloat(
                "baseline_update_tau", 1.0, above=0.0),
            baseline_quiet_time=config.getfloat(
                "baseline_quiet_time", 0.5, minval=0.0),
            baseline_max_deviation=config.getfloat(
                "baseline_max_deviation", 250.0, above=0.0),
            baseline_noise_factor=config.getfloat(
                "baseline_noise_factor", 6.0, above=0.0),
            noise_tau=config.getfloat("noise_tau", 3.0, above=0.0),
            extrusion_epsilon=config.getfloat(
                "extrusion_epsilon", 0.0001, above=0.0),
            xy_epsilon=config.getfloat("xy_epsilon", 0.01, above=0.0),
            minimum_flow=config.getfloat("minimum_flow", 0.5, minval=0.0),
            transient_time=config.getfloat(
                "transient_time", 0.30, minval=0.0),
            flow_change_ratio=config.getfloat(
                "flow_change_ratio", 0.10, above=0.0),
            max_confident_noise=config.getfloat(
                "max_confident_noise", 100.0, above=0.0))
        self.clients = []
        self.dump_clients = []
        self.latest = None
        self.last_callback_time = None
        self.extruder_providers = {}
        self.extruders = {}
        self.toolhead_provider = None
        self.last_extruder = None
        self.profile_manager = None
        self.operation_owner = None
        self.load_cell = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        webhooks = self.printer.lookup_object("webhooks", None)
        if webhooks is not None:
            webhooks.register_mux_endpoint(
                "extrusion_force/dump", "name", self.name,
                self._handle_dump_request)

    def _handle_ready(self):
        self.load_cell = self.printer.lookup_object(self.load_cell_name)
        status = self.load_cell.get_status(self.reactor.monotonic())
        if not status.get("is_calibrated", False):
            raise self.printer.config_error(
                "extrusion_force_monitor requires a calibrated load cell")
        self.load_cell.add_client(self._handle_sample)
        toolhead = self.printer.lookup_object("toolhead")
        self.toolhead_provider = TrapQMotionProvider(
            "toolhead", toolhead.get_trapq())
        for name, extruder in self.printer.lookup_objects("extruder"):
            self.extruders[name] = extruder
            self.extruder_providers[name] = TrapQMotionProvider(
                name, extruder.get_trapq(), extruder=True)
        self.profile_manager = self.printer.lookup_object(
            "extrusion_force_profile_manager", None)

    def _handle_dump_request(self, web_request):
        from .bulk_sensor import BatchWebhooksClient
        client = BatchWebhooksClient(web_request)
        self.dump_clients.append(client.handle_batch)
        web_request.send({"header": (
            "print_time", "force_g", "expected_force_g", "excess_force_g",
            "flow_mm3_s", "temperature", "motion_state", "confidence")})

    def _motion_at(self, print_time):
        moving = []
        states = {}
        for name, provider in self.extruder_providers.items():
            state = provider.get_state(print_time)
            if state is None:
                continue
            states[name] = state
            if abs(state["e_velocity"]) > self.processor.classifier.extrusion_epsilon:
                moving.append(state)
        if moving:
            state = max(moving, key=lambda item: abs(item["e_velocity"]))
            self.last_extruder = state["extruder"]
        elif self.last_extruder in states:
            state = states[self.last_extruder]
        elif states:
            state = states[sorted(states)[0]]
            self.last_extruder = state["extruder"]
        else:
            state = {"extruder": None, "e_position": None,
                     "e_velocity": 0.0}
        xy_state = (self.toolhead_provider.get_state(print_time)
                    if self.toolhead_provider is not None else None)
        state["xy_velocity"] = (xy_state or {}).get("xy_velocity", 0.0)
        return state

    def _handle_sample(self, sample):
        print_time = sample["print_time"]
        motion = self._motion_at(print_time)
        extruder_name = motion["extruder"]
        extruder = self.extruders.get(extruder_name)
        eventtime = self.reactor.monotonic()
        temperature = target = 0.0
        filament_area = 0.0
        if extruder is not None:
            status = extruder.get_status(eventtime)
            temperature = status["temperature"]
            target = status["target"]
            filament_area = extruder.filament_area
        flow = max(0.0, motion["e_velocity"] * filament_area)
        observation = dict(sample)
        observation.update(motion)
        observation.update({
            "flow_mm3_s": flow,
            "temperature": temperature,
            "target_temperature": target,
        })
        profile = (self.profile_manager.get_active(extruder_name)
                   if self.profile_manager is not None else None)
        self.latest = self.processor.process(observation, profile)
        if (self.last_callback_time is None
                or print_time - self.last_callback_time
                >= self.callback_interval):
            self.last_callback_time = print_time
            self._publish(self.latest)

    def _publish(self, state):
        for callback in list(self.clients):
            callback(dict(state))
        data = tuple(state.get(key) for key in (
            "print_time", "force_g", "expected_force_g", "excess_force_g",
            "flow_mm3_s", "temperature", "motion_state", "confidence"))
        message = {"data": [data]}
        for callback in list(self.dump_clients):
            if callback(message) is False:
                self.dump_clients.remove(callback)

    def add_client(self, callback):
        if callback not in self.clients:
            self.clients.append(callback)

    def remove_client(self, callback):
        if callback in self.clients:
            self.clients.remove(callback)

    def get_latest_state(self):
        return dict(self.latest) if self.latest is not None else None

    def get_expected_force(self, extruder, flow, temperature):
        if self.profile_manager is None:
            return None
        profile = self.profile_manager.get_active(extruder)
        if profile is None:
            return None
        return profile.expected_force(flow, temperature)

    def get_excess_force(self):
        if self.latest is None:
            return None
        return self.latest["excess_force_g"]

    def get_active_profile(self, extruder):
        if self.profile_manager is None:
            return None
        return self.profile_manager.get_active(extruder)

    def claim_operation(self, owner):
        if self.operation_owner is not None and self.operation_owner != owner:
            raise self.printer.command_error(
                "Load-cell operation already active: %s"
                % (self.operation_owner,))
        self.operation_owner = owner

    def release_operation(self, owner):
        if self.operation_owner == owner:
            self.operation_owner = None

    def get_status(self, eventtime):
        if self.latest is None:
            return {
                "enabled": True,
                "extruder": None,
                "profile": None,
                "state": IDLE,
                "force_g": 0.0,
                "expected_force_g": None,
                "excess_force_g": None,
                "flow_mm3_s": 0.0,
                "e_velocity": 0.0,
                "temperature": 0.0,
                "noise_g": 0.0,
                "confidence": 0.0,
                "operation": self.operation_owner,
            }
        state = self.latest
        return {
            "enabled": True,
            "extruder": state["extruder"],
            "profile": state["profile"],
            "state": state["motion_state"],
            "force_g": state["force_control_g"],
            "expected_force_g": state["expected_dynamic_force_g"],
            "excess_force_g": state["excess_force_g"],
            "flow_mm3_s": state["flow_mm3_s"],
            "e_velocity": state["e_velocity"],
            "temperature": state["temperature"],
            "noise_g": state["noise_g"],
            "confidence": state["confidence"],
            "operation": self.operation_owner,
        }


def load_config(config):
    return ExtrusionForceMonitor(config)
