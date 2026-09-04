# Support generic analog inputs
#
# Copyright (C) 2025  Russell Cloran <rcloran@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

SAMPLE_COUNT = 8  # Take 8 subsamples within the MCU
SAMPLE_TIME = 0.001  # with a 0.001s gap between each
REPORT_TIME = 1.000  # and report their average every 1s


class LimitHelper:
    def __init__(self, config, idx):
        main_key = "limit%d" % (idx, )
        over_key = "over_%s_gcode" % (main_key, )
        under_key = "under_%s_gcode" % (main_key, )

        printer = config.get_printer()
        self.limit = config.getfloat(main_key)
        self.gcode = printer.lookup_object('gcode')
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.over_template = gcode_macro.load_template(config, over_key, '')
        self.under_template = gcode_macro.load_template(config, under_key, '')

    def callback(self, read_time, value, last_value):
        if value < self.limit and last_value >= self.limit:
            self.under_template.run_gcode_from_command()
        elif value > self.limit and last_value <= self.limit:
            self.over_template.run_gcode_from_command()


class AnalogMeasurement:
    def __init__(self, config, name, option_prefix="", report_time=REPORT_TIME,
                 initial_value=None, value_callback=None, pin_option=None):
        self.name = name
        self.value_callback = value_callback
        self.scale = config.getfloat(option_prefix + "scale", 1.0)
        self.offset = config.getfloat(option_prefix + "offset", 0.0)
        self.last_value = initial_value

        report_time = config.getfloat(option_prefix + "report_time",
                                      report_time, above=0.)
        sample_time = config.getfloat(option_prefix + "sample_time",
                                      SAMPLE_TIME, above=0.)
        sample_count = config.getint(option_prefix + "sample_count",
                                     SAMPLE_COUNT, minval=1)
        if sample_time * sample_count >= report_time:
            raise config.error(
                "Option '%sreport_time' in section '%s' must be greater "
                "than %ssample_time * %ssample_count" % (
                    option_prefix, config.get_name(), option_prefix,
                    option_prefix))

        printer = config.get_printer()
        ppins = printer.lookup_object("pins")
        if pin_option is None:
            pin_option = option_prefix + "pin"
        self.mcu_adc = ppins.setup_pin(
            "adc", config.get(pin_option))
        self.mcu_adc.setup_adc_callback(self.adc_callback)
        self.mcu_adc.setup_adc_sample(report_time, sample_time, sample_count)
        query_adc = printer.load_object(config, "query_adc")
        query_adc.register_adc(self.name, self.mcu_adc)

    def adc_callback(self, samples):
        read_time, read_value = samples[-1]
        last_value = self.last_value
        value = read_value * self.scale + self.offset
        if self.value_callback is not None:
            self.value_callback(read_time, value, last_value)
        self.last_value = value

    def get_value(self):
        return self.last_value


class PrinterAnalogInput:
    def __init__(self, config):
        self.name = config.get_name()
        self.unit = config.get("unit", "")
        self.decimal_places = config.getint(
            "decimal_places", 2, minval=0, maxval=6)
        self._limit_helpers = []
        for i in range(1, 1000):
            if config.get("limit%d" % (i, ), None) is None:
                break
            self._limit_helpers.append(LimitHelper(config, i))

        self.measurement = AnalogMeasurement(
            config, self.name, initial_value=0.0,
            value_callback=self._value_callback, pin_option="sensor_pin")
        self.scale = self.measurement.scale
        self.offset = self.measurement.offset
        self.mcu_adc = self.measurement.mcu_adc

    @property
    def last_value(self):
        return self.measurement.get_value()

    @last_value.setter
    def last_value(self, value):
        self.measurement.last_value = value

    def adc_callback(self, samples):
        self.measurement.adc_callback(samples)

    def _value_callback(self, read_time, value, last_value):
        for helper in self._limit_helpers:
            helper.callback(read_time, value, last_value)

    def stats(self, eventtime):
        msg = "%s: value=%.1f%s" % (self.name, self.last_value, self.unit)
        return False, msg

    def get_status(self, eventtime):
        return {
            "value": round(self.last_value, self.decimal_places),
            "unit": self.unit,
            "decimal_places": self.decimal_places,
        }


def load_config_prefix(config):
    return PrinterAnalogInput(config)
