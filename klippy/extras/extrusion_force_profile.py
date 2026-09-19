# Persistent force/flow/temperature profiles for extrusion-force monitoring
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import json
import math


def _linear_interpolate(x, x0, y0, x1, y1):
    if x1 == x0:
        return y0
    ratio = (x - x0) / float(x1 - x0)
    return y0 + ratio * (y1 - y0)


def _linear_fit(points):
    """Return slope, intercept, and squared residual error."""
    count = len(points)
    mean_x = sum(point[0] for point in points) / float(count)
    mean_y = sum(point[1] for point in points) / float(count)
    variance = sum((point[0] - mean_x) ** 2 for point in points)
    if variance <= 0.0:
        return 0.0, mean_y, float("inf")
    slope = sum((point[0] - mean_x) * (point[1] - mean_y)
                for point in points) / variance
    intercept = mean_y - slope * mean_x
    error = sum((slope * point[0] + intercept - point[1]) ** 2
                for point in points)
    return slope, intercept, error


def detect_knee(points, minimum_slope_ratio=2.0,
                minimum_fit_improvement=0.25):
    """Find a two-line breakpoint without inventing one for linear data."""
    ordered = sorted((float(flow), float(force)) for flow, force in points)
    if len(ordered) < 5:
        return None
    single_slope, single_intercept, single_error = _linear_fit(ordered)
    best = None
    for split in range(2, len(ordered) - 1):
        lower = ordered[:split + 1]
        upper = ordered[split:]
        low_slope, _, low_error = _linear_fit(lower)
        high_slope, _, high_error = _linear_fit(upper)
        if low_slope <= 0.0 or high_slope < low_slope * minimum_slope_ratio:
            continue
        error = low_error + high_error
        if best is None or error < best[0]:
            best = (error, ordered[split][0], low_slope, high_slope)
    if best is None:
        return None
    # A breakpoint must materially improve the fit, not merely split noise in
    # an otherwise linear curve.
    if (single_error <= 0.0
            or 1.0 - best[0] / single_error < minimum_fit_improvement):
        return None
    return {
        "flow": best[1],
        "lower_slope": best[2],
        "upper_slope": best[3],
        "fit_improvement": 1.0 - best[0] / single_error,
    }


class ForceProfile:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.section_name = config.get_name()
        name_parts = self.section_name.split(None, 1)
        if len(name_parts) != 2:
            raise config.error("extrusion_force_profile requires a name")
        self.name = name_parts[1]
        self.extruders = config.getlist("extruder", ("extruder",))
        if not self.extruders or any(not name for name in self.extruders):
            raise config.error(
                "extrusion_force_profile requires at least one extruder")
        if len(set(self.extruders)) != len(self.extruders):
            raise config.error(
                "Duplicate extruder in [%s]" % (self.section_name,))
        # Keep the singular attribute for compatibility with existing status
        # consumers. Commands that operate hardware use resolve_extruder().
        self.extruder = self.extruders[0]
        self.valid_for = config.get("valid_for")
        self.nozzle_diameter = config.getfloat("nozzle_diameter", above=0.0)
        for moved_option in ("material", "max_material_temperature"):
            if config.get(moved_option, None) is not None:
                destination = "[%s]" % (
                    "filament_profile %s" % (self.valid_for,))
                raise config.error(
                    "%s in [%s] belongs in %s"
                    % (moved_option, self.section_name, destination))
        self.filament_profile = None
        self.compatibility_errors = {}
        self.response_tau_rise = config.getfloat(
            "response_tau_rise", 0.25, above=0.0)
        self.response_tau_fall = config.getfloat(
            "response_tau_fall", 0.5, above=0.0)
        self.temperature_tolerance = config.getfloat(
            "temperature_tolerance", 2.0, minval=0.0)
        self.flow_safety_factor = config.getfloat(
            "flow_safety_factor", 0.85, above=0.0, maxval=1.0)
        self.points = self._load_points(config.get("calibration_data", ""))
        self.flow_limits = self._load_flow_limits(
            config.get("recommended_max_flow", ""))
        self.physical_flow_limits = self._load_flow_limits(
            config.get("physical_flow_limit", ""))
        try:
            lower_bounds = json.loads(
                config.get("physical_flow_lower_bound", "[]"))
            self.physical_flow_lower_bounds = set(
                float(temperature) for temperature in lower_bounds)
        except (TypeError, ValueError):
            raise self.printer.config_error(
                "Invalid physical_flow_lower_bound in [%s]"
                % (self.section_name,))
        self.manager = self.printer.lookup_object(
            "extrusion_force_profile_manager", None)
        if self.manager is None:
            self.manager = ForceProfileManager(config)
            self.printer.add_object(
                "extrusion_force_profile_manager", self.manager)
        self.manager.add_profile(self)

    @property
    def material(self):
        if self.filament_profile is None:
            return ""
        return self.filament_profile.material

    @property
    def max_material_temperature(self):
        if self.filament_profile is None:
            return None
        return self.filament_profile.max_material_temperature

    @property
    def filament_diameter(self):
        if self.filament_profile is None:
            return None
        return self.filament_profile.filament_diameter

    def get_filament_area(self, nominal_area):
        if (self.filament_profile is None
                or self.filament_profile.filament_area is None):
            return nominal_area
        return self.filament_profile.filament_area

    def bind_filament_profile(self, filament_profile):
        self.filament_profile = filament_profile

    def check_extruder(self, extruder_name, extruder):
        errors = []
        if not math.isclose(
                self.nozzle_diameter, extruder.nozzle_diameter,
                rel_tol=0.0, abs_tol=1e-6):
            errors.append(
                "nozzle_diameter %.6g does not match %.6g"
                % (self.nozzle_diameter, extruder.nozzle_diameter))
        self.compatibility_errors[extruder_name] = errors
        return errors

    def compatibility_error(self, extruder):
        errors = self.compatibility_errors.get(extruder)
        if errors is None:
            return "extruder compatibility has not been checked"
        if errors:
            return "; ".join(errors)
        return None

    def _load_points(self, value):
        if not value:
            return []
        try:
            raw_points = json.loads(value)
            points = []
            for point in raw_points:
                points.append({
                    "temperature": float(point["temperature"]),
                    "flow": float(point["flow"]),
                    "mean_force": float(point["mean_force"]),
                    "force_sigma": float(point.get("force_sigma", 0.0)),
                    "sample_count": int(point.get("sample_count", 0)),
                })
            return points
        except (TypeError, ValueError, KeyError) as error:
            raise self.printer.config_error(
                "Invalid calibration_data in [%s]: %s"
                % (self.section_name, error))

    def _load_flow_limits(self, value):
        if not value:
            return {}
        try:
            raw_limits = json.loads(value)
            return {float(temp): float(flow)
                    for temp, flow in raw_limits.items()}
        except (TypeError, ValueError) as error:
            raise self.printer.config_error(
                "Invalid recommended_max_flow in [%s]: %s"
                % (self.section_name, error))

    def _curves(self):
        curves = {}
        for point in self.points:
            curves.setdefault(point["temperature"], []).append(point)
        for curve in curves.values():
            curve.sort(key=lambda point: point["flow"])
        return curves

    def _curve_force(self, curve, flow):
        if not curve or flow < curve[0]["flow"] or flow > curve[-1]["flow"]:
            return None
        for point in curve:
            if math.isclose(flow, point["flow"], rel_tol=0.0,
                            abs_tol=1e-9):
                return point["mean_force"]
        for lower, upper in zip(curve, curve[1:]):
            if lower["flow"] <= flow <= upper["flow"]:
                return _linear_interpolate(
                    flow, lower["flow"], lower["mean_force"],
                    upper["flow"], upper["mean_force"])
        return None

    def expected_force(self, flow, temperature):
        curves = self._curves()
        temperatures = sorted(curves)
        if not temperatures:
            return None
        if len(temperatures) == 1:
            temp = temperatures[0]
            if abs(temperature - temp) > self.temperature_tolerance:
                return None
            return self._curve_force(curves[temp], flow)
        if temperature < temperatures[0]:
            if temperatures[0] - temperature > self.temperature_tolerance:
                return None
            temperature = temperatures[0]
        elif temperature > temperatures[-1]:
            if temperature - temperatures[-1] > self.temperature_tolerance:
                return None
            temperature = temperatures[-1]
        for temp in temperatures:
            if math.isclose(temperature, temp, rel_tol=0.0,
                            abs_tol=1e-9):
                return self._curve_force(curves[temp], flow)
        for lower_temp, upper_temp in zip(temperatures, temperatures[1:]):
            if lower_temp <= temperature <= upper_temp:
                lower_force = self._curve_force(curves[lower_temp], flow)
                upper_force = self._curve_force(curves[upper_temp], flow)
                if lower_force is None or upper_force is None:
                    return None
                return _linear_interpolate(
                    temperature, lower_temp, lower_force,
                    upper_temp, upper_force)
        return None

    def get_recommended_max_flow(self, temperature):
        temperatures = sorted(self.flow_limits)
        if not temperatures:
            return None
        if len(temperatures) == 1:
            temp = temperatures[0]
            if abs(temperature - temp) > self.temperature_tolerance:
                return None
            return self.flow_limits[temp]
        if temperature < temperatures[0]:
            if temperatures[0] - temperature > self.temperature_tolerance:
                return None
            temperature = temperatures[0]
        elif temperature > temperatures[-1]:
            if temperature - temperatures[-1] > self.temperature_tolerance:
                return None
            temperature = temperatures[-1]
        for temp in temperatures:
            if math.isclose(temperature, temp, rel_tol=0.0,
                            abs_tol=1e-9):
                return self.flow_limits[temp]
        for lower_temp, upper_temp in zip(temperatures, temperatures[1:]):
            if lower_temp <= temperature <= upper_temp:
                return _linear_interpolate(
                    temperature, lower_temp, self.flow_limits[lower_temp],
                    upper_temp, self.flow_limits[upper_temp])
        return None

    def get_physical_flow_limit(self, temperature):
        if temperature in self.physical_flow_limits:
            return {
                "flow": self.physical_flow_limits[temperature],
                "is_lower_bound": (
                    temperature in self.physical_flow_lower_bounds),
            }
        return None

    def replace_calibration(self, points, flow_limits,
                            response_tau_rise=None, response_tau_fall=None,
                            physical_flow_limits=None,
                            physical_flow_lower_bounds=None):
        self.points = list(points)
        self.flow_limits = dict(flow_limits)
        if response_tau_rise is not None:
            self.response_tau_rise = response_tau_rise
        if response_tau_fall is not None:
            self.response_tau_fall = response_tau_fall
        if physical_flow_limits is not None:
            self.physical_flow_limits = dict(physical_flow_limits)
        if physical_flow_lower_bounds is not None:
            self.physical_flow_lower_bounds = set(
                physical_flow_lower_bounds)
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.section_name, "calibration_data",
                       json.dumps(self.points, separators=(",", ":")))
        limits = {str(temp): flow for temp, flow in self.flow_limits.items()}
        configfile.set(self.section_name, "recommended_max_flow",
                       json.dumps(limits, separators=(",", ":")))
        physical_limits = {
            str(temp): flow
            for temp, flow in self.physical_flow_limits.items()}
        configfile.set(self.section_name, "physical_flow_limit",
                       json.dumps(physical_limits, separators=(",", ":")))
        configfile.set(
            self.section_name, "physical_flow_lower_bound",
            json.dumps(sorted(self.physical_flow_lower_bounds),
                       separators=(",", ":")))
        configfile.set(self.section_name, "response_tau_rise",
                       "%.6f" % (self.response_tau_rise,))
        configfile.set(self.section_name, "response_tau_fall",
                       "%.6f" % (self.response_tau_fall,))

    def supports_extruder(self, extruder):
        return extruder in self.extruders

    def resolve_extruder(self, gcmd):
        extruder = gcmd.get("EXTRUDER", None)
        if extruder is None:
            if len(self.extruders) != 1:
                raise gcmd.error(
                    "Profile '%s' applies to multiple extruders (%s); "
                    "specify EXTRUDER"
                    % (self.name, ", ".join(self.extruders)))
            extruder = self.extruder
        if not self.supports_extruder(extruder):
            raise gcmd.error(
                "Profile '%s' does not apply to extruder '%s' (allowed: %s)"
                % (self.name, extruder, ", ".join(self.extruders)))
        incompatibility = self.compatibility_error(extruder)
        if incompatibility is not None:
            raise gcmd.error(
                "Profile '%s' is incompatible with extruder '%s': %s"
                % (self.name, extruder, incompatibility))
        return extruder

    def get_status(self, eventtime):
        return {
            "name": self.name,
            "extruder": self.extruder,
            "extruders": list(self.extruders),
            "valid_for": self.valid_for,
            "material": self.material,
            "nozzle_diameter": self.nozzle_diameter,
            "filament_diameter": self.filament_diameter,
            "max_material_temperature": self.max_material_temperature,
            "compatible_extruders": [
                extruder for extruder in self.extruders
                if not self.compatibility_errors.get(extruder, [True])
            ],
            "incompatible_extruders": {
                extruder: "; ".join(errors)
                for extruder, errors in self.compatibility_errors.items()
                if errors
            },
            "calibration_points": len(self.points),
            "response_tau_rise": self.response_tau_rise,
            "response_tau_fall": self.response_tau_fall,
            "recommended_max_flow": dict(self.flow_limits),
            "physical_flow_limit": dict(self.physical_flow_limits),
            "physical_flow_lower_bound": sorted(
                self.physical_flow_lower_bounds),
        }


class ForceProfileManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.profiles = {}
        self.active = {}
        self.filament_manager = None
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_EXTRUSION_FORCE_PROFILE", self.cmd_SET_PROFILE,
            desc="Select a calibrated extrusion-force profile")

    def add_profile(self, profile):
        key = profile.name.lower()
        if key in self.profiles:
            raise self.printer.config_error(
                "Duplicate extrusion force profile '%s'" % (profile.name,))
        self.profiles[key] = profile

    def _handle_connect(self):
        self.filament_manager = self.printer.lookup_object(
            "filament_profile_manager", None)
        if self.filament_manager is None:
            raise self.printer.config_error(
                "extrusion_force_profile requires at least one "
                "filament_profile")
        for profile in self.profiles.values():
            filament = self.filament_manager.get_profile(profile.valid_for)
            if filament is None:
                raise self.printer.config_error(
                    "Unknown filament profile '%s' referenced by [%s]"
                    % (profile.valid_for, profile.section_name))
            profile.bind_filament_profile(filament)
            for extruder_name in profile.extruders:
                extruder = self.printer.lookup_object(extruder_name, None)
                if extruder is None:
                    raise self.printer.config_error(
                        "Unknown extruder '%s' referenced by [%s]"
                        % (extruder_name, profile.section_name))
                profile.check_extruder(extruder_name, extruder)

    def get_profile(self, name):
        if name is None:
            return None
        return self.profiles.get(name.lower())

    def get_active(self, extruder):
        return self.active.get(extruder)

    def find_for_filament(self, filament, extruder, gcmd):
        matching = [
            profile for profile in self.profiles.values()
            if profile.valid_for.lower() == filament.name.lower()
            and profile.supports_extruder(extruder)
            and profile.compatibility_error(extruder) is None
        ]
        if len(matching) > 1:
            raise gcmd.error(
                "Multiple compatible extrusion force profiles for filament "
                "'%s' and extruder '%s': %s"
                % (filament.name, extruder,
                   ", ".join(profile.name for profile in matching)))
        return matching[0] if matching else None

    def activate_for_filament(self, extruder, profile):
        if profile is None:
            self.active.pop(extruder, None)
            profile_name = None
        else:
            self.active[extruder] = profile
            profile_name = profile.name
        self.printer.send_event(
            "extrusion_force:profile_changed", extruder, profile_name)

    def cmd_SET_PROFILE(self, gcmd):
        name = gcmd.get("PROFILE")
        profile = self.get_profile(name)
        if profile is None:
            raise gcmd.error("Unknown extrusion force profile '%s'" % (name,))
        extruder = gcmd.get("EXTRUDER", None)
        if extruder is None:
            extruders = profile.extruders
        else:
            if not profile.supports_extruder(extruder):
                raise gcmd.error(
                    "Profile '%s' does not apply to extruder '%s' "
                    "(allowed: %s)"
                    % (profile.name, extruder,
                       ", ".join(profile.extruders)))
            extruders = (extruder,)
        for extruder in extruders:
            incompatibility = profile.compatibility_error(extruder)
            if incompatibility is not None:
                raise gcmd.error(
                    "Profile '%s' is incompatible with extruder '%s': %s"
                    % (profile.name, extruder, incompatibility))
        for extruder in extruders:
            self.activate_for_filament(extruder, profile)
            if self.filament_manager is not None:
                self.filament_manager.activate(
                    extruder, profile.filament_profile)
        gcmd.respond_info(
            "Extrusion force profile for %s: %s"
            % (", ".join(extruders), profile.name))


def load_config_prefix(config):
    return ForceProfile(config)
