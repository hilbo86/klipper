# Printer-independent filament identities for extrusion-force profiles
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math


class FilamentProfile:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.section_name = config.get_name()
        name_parts = self.section_name.split(None, 1)
        if len(name_parts) != 2:
            raise config.error("filament_profile requires a name")
        self.name = name_parts[1]
        self.material = config.get("material")
        self.max_material_temperature = config.getfloat(
            "max_material_temperature", None, above=0.0)
        self.filament_diameter = config.getfloat(
            "filament_diameter", None, above=0.0)
        self.filament_area = (
            math.pi * (self.filament_diameter * 0.5) ** 2
            if self.filament_diameter is not None else None)
        self.manager = self.printer.lookup_object(
            "filament_profile_manager", None)
        if self.manager is None:
            self.manager = FilamentProfileManager(config)
            self.printer.add_object(
                "filament_profile_manager", self.manager)
        self.manager.add_profile(self)

    def get_status(self, eventtime):
        return {
            "name": self.name,
            "material": self.material,
            "max_material_temperature": self.max_material_temperature,
            "filament_diameter": self.filament_diameter,
        }


class FilamentProfileManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.profiles = {}
        self.active = {}
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_FILAMENT_PROFILE", self.cmd_SET_FILAMENT_PROFILE,
            desc="Select a printer-independent filament profile")

    def add_profile(self, profile):
        key = profile.name.lower()
        if key in self.profiles:
            raise self.printer.config_error(
                "Duplicate filament profile '%s'" % (profile.name,))
        self.profiles[key] = profile

    def get_profile(self, name):
        if name is None:
            return None
        return self.profiles.get(name.lower())

    def get_active(self, extruder):
        return self.active.get(extruder)

    def get_filament_area(self, extruder, nominal_area):
        profile = self.get_active(extruder)
        if profile is None or profile.filament_area is None:
            return nominal_area
        return profile.filament_area

    def _resolve_extruder(self, gcmd):
        extruder_name = gcmd.get("EXTRUDER", None)
        if extruder_name is not None:
            extruder = self.printer.lookup_object(extruder_name, None)
            if (extruder is None
                    or not hasattr(extruder, "filament_area")
                    or not hasattr(extruder, "get_heater")):
                raise gcmd.error("Unknown extruder '%s'" % (extruder_name,))
            return extruder_name
        toolhead = self.printer.lookup_object("toolhead")
        return toolhead.get_extruder().get_name()

    def activate(self, extruder, profile):
        self.active[extruder] = profile
        self.printer.send_event(
            "filament_profile:changed", extruder, profile.name)

    def cmd_SET_FILAMENT_PROFILE(self, gcmd):
        name = gcmd.get("PROFILE")
        filament = self.get_profile(name)
        if filament is None:
            raise gcmd.error("Unknown filament profile '%s'" % (name,))
        extruder = self._resolve_extruder(gcmd)
        force_manager = self.printer.lookup_object(
            "extrusion_force_profile_manager", None)
        force_profile = None
        if force_manager is not None:
            force_profile = force_manager.find_for_filament(
                filament, extruder, gcmd)
        self.activate(extruder, filament)
        if force_manager is not None:
            force_manager.activate_for_filament(extruder, force_profile)
        if force_profile is None:
            force_text = "no compatible extrusion-force profile"
        else:
            force_text = "extrusion-force profile %s" % (force_profile.name,)
        gcmd.respond_info(
            "Filament profile for %s: %s (%s)"
            % (extruder, filament.name, force_text))

    def get_status(self, eventtime):
        return {
            "active": {
                extruder: profile.name
                for extruder, profile in self.active.items()
            },
        }


def load_config_prefix(config):
    return FilamentProfile(config)
