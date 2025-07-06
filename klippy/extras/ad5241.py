# AD5241 digipot with 2 digital outputs code
#
# Copyright (C) 2025  Timo Hilbig <timo@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bus

AD5241_CHIP_ADDR = 44
class AD5241_digital_out:
    def __init__(self, main, pin_params):
        #self._printer = main._printer
        #self._reactor = main._printer.get_reactor()
        #self._i2c = main._i2c
        self._mcu = main._mcu
        #self._oid = None
        p = pin_params['pin']
        if p not in ['O1', 'O2']:
            raise self._printer.config_error(
                f"pin name {p} not supported as digital_out on AD5241")
        self._pin = p
        self._invert = pin_params['invert']
        self._start_value = self._shutdown_value = self._invert
        self._max_duration = 2.
        self._last_clock = 0
        self._set_cmd = main._set_digital

    def get_mcu(self):
        return self._mcu
    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration
    def setup_start_value(self, start_value, shutdown_value):
        self._start_value = (not not start_value) ^ self._invert
        self._shutdown_value = (not not shutdown_value) ^ self._invert

    def set_digital(self, print_time, value):
        clock = self._mcu.print_time_to_clock(print_time)
        self._set_cmd(self._pin, (not not value) ^ self._invert,
                      minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock

class AD5241_pwm:
    def __init__(self, main, pin_params):
        self._mcu = main._mcu
        p = pin_params['pin']
        if p != 'W1':
            raise self._printer.config_error(
                f"pin name {p} not supported as pwm_out on AD5241")
        self._pin = p
        self._invert = pin_params['invert']
        self._start_value = self._shutdown_value = float(self._invert)
        self._max_duration = 2.
        self._last_clock = 0
        self._pwm_max = 0.
        self._set_cmd = main._set_wiper

    def get_mcu(self):
        return self._mcu
    def setup_max_duration(self, max_duration):
        self._max_duration = max_duration
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        pass
    def setup_start_value(self, start_value, shutdown_value):
        if self._invert:
            start_value = 1. - start_value
            shutdown_value = 1. - shutdown_value
        self._start_value = max(0., min(1., start_value))
        self._shutdown_value = max(0., min(1., shutdown_value))

    def set_pwm(self, print_time, value):
        if self._invert:
            value = 1. - value
        v = int(max(0., min(1., value)) * 255)
        clock = self._mcu.print_time_to_clock(print_time)
        self._set_cmd(v, minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock

class AD5241:
    def __init__(self, config):
        self._printer = config.get_printer()
        self._name = config.get_name().split()[1]
        self._i2c = bus.MCU_I2C_from_config(config,
            default_addr=AD5241_CHIP_ADDR)
        i2c_addr = self._i2c.get_i2c_address()
        if i2c_addr < 44 or i2c_addr > 47:
            raise config.error("ad5241 address must be between 44 and 47")
        self._mcu = self._i2c.get_mcu()
        # Register setup_pin
        ppins = self._printer.lookup_object('pins')
        ppins.register_chip(self._name, self)
        self.digi = {'O1':0, 'O2':0}
        self.wiper = 0

    def setup_pin(self, pin_type, pin_params):
        pcs = {'digital_out': AD5241_digital_out, 'pwm': AD5241_pwm}
        if pin_type not in pcs:
            raise self._printer.config_error(
                f"pin type {pin_type} not supported on AD5241")
        return pcs[pin_type](self, pin_params)

    def _set_digital(self, pin, value, minclock=0, reqclock=0):
        value = not not value
        if self.digi[pin] == value:
            return
        self.digi[pin] = value
        self._i2c.i2c_write([(self.digi['O1'] << 4) | (self.digi['O2'] << 3)],
                            minclock, reqclock)

    def _set_wiper(self, value, minclock=0, reqclock=0):
        if self.wiper == value:
            return
        self.wiper = value
        self._i2c.i2c_write([(self.digi['O1'] << 4) | (self.digi['O2'] << 3),
                             value], minclock, reqclock)

def load_config_prefix(config):
    return AD5241(config)
