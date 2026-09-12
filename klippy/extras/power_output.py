# Output pin with current monitoring
#
# Copyright (C) 2026  Klipper developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import analog_input, output_pin


CURRENT_REPORT_TIME = 0.250


class PrinterPowerOutput(output_pin.PrinterOutputPin):
    def __init__(self, config):
        super().__init__(config)
        current_name = config.get_name() + ":current"
        self.current = analog_input.AnalogMeasurement(
            config, current_name, option_prefix="current_",
            report_time=CURRENT_REPORT_TIME)

    def get_status(self, eventtime):
        status = super().get_status(eventtime)
        status["current"] = self.current.get_value()
        status["current_unit"] = "A"
        return status


def load_config_prefix(config):
    return PrinterPowerOutput(config)
