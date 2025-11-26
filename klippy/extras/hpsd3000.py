# Support for i2c based pressure/temperature sensors
#
# Copyright (C) 2025  Timo Hilbig <timo.hilbig@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

REPORT_TIME = 1.
HPSD_CHIP_ADDR = 0x78

HPSD_PDAT_MIN = 3277.
HPSD_PDAT_MAX = 29491.
HPSD_P_MAX = 20.
HPSD_P_MIN = 0.
HPSD_CONST_S = (HPSD_PDAT_MAX - HPSD_PDAT_MIN) / (HPSD_P_MAX - HPSD_P_MIN)
HPSD_CONST_T = 220.
HPSD_OFFSET_T = 12000


class HPSD3000:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=HPSD_CHIP_ADDR, default_speed=100000)
        self.mcu = self.i2c.get_mcu()

        self.temp = self.pressure  = 0.
        self.min_temp = self.max_temp = 0.
        self.sample_timer = None
        self.printer.add_object("hpsd3000 " + self.name, self)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_hpsd()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def _init_hpsd(self):
        self.sample_timer = self.reactor.register_timer(self._sample_hpsd)

    def _sample_hpsd(self, eventtime):
        try:
            params = self.i2c.i2c_read([], 4)  # ggf ([0], 4)
            data = bytearray(params['response'])
        except Exception:
            logging.exception("HPSD3000: Error reading data")
            self.temp = self.pressure = .0
            return self.reactor.NEVER
        self.pressure = (int.from_bytes(data[:2], 'big') - HPSD_PDAT_MIN) / HPSD_CONST_S - HPSD_P_MIN
        self.temp = (int.from_bytes(data[2:], 'big') - HPSD_OFFSET_T) / HPSD_CONST_T
        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + REPORT_TIME

    def get_status(self, eventtime):
        data = {
            'temperature': round(self.temp, 2),
            'pressure': self.pressure
        }
        return data


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("HPSD3000", HPSD3000)
