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
            raise self._printer.config_error("pin name %s not supported as digital_out on AD5241" % (p,))
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
        self._set_cmd(self._pin, (not not value) ^ self._invert, minclock=self._last_clock, reqclock=clock)
        self._last_clock = clock
 
class AD5241_pwm:
    def __init__(self, main, pin_params):
        #self._printer = main._printer
        #self._reactor = main._printer.get_reactor()
        #self._i2c = main._i2c
        self._mcu = main._mcu
        #self._oid = None
        p = pin_params['pin']
        if p != 'W1':
            raise self._printer.config_error("pin name %s not supported as pwm_out on AD5241" % (p,))
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
            raise self._printer.config_error("pin type %s not supported on AD5241" % (pin_type,))
        return pcs[pin_type](self, pin_params)
    
    def _set_digital(self, pin, value, minclock=0, reqclock=0):
        value = not not value
        if self.digi[pin] == value:
            return
        self.digi[pin] = value
        self._i2c.i2c_write([(self.digi['O1'] << 4) | (self.digi['O2'] << 3)], minclock, reqclock)

    def _set_wiper(self, value, minclock=0, reqclock=0):
        if self.wiper == value:
            return
        self.wiper = value
        self._i2c.i2c_write([(self.digi['O1'] << 4) | (self.digi['O2'] << 3), value], minclock, reqclock)
   
'''
pin_params: dict(   chip -> Chip-Objekt
                    chip_name -> Bezeichnung des Chips
                    pin -> Bezeichnung des Pins, wie in der Config spezifiziert
                    invert -> 1 oder 0
                    pullup -> -1, 1 oder 0
                )
'''
 
 
#class ad5241:
#    def __init__(self, config):
#        self.i2c = bus.MCU_I2C_from_config(config, default_addr=44)
#        i2c_addr = self.i2c.get_i2c_address()
#        if i2c_addr < 44 or i2c_addr > 47:
#            raise config.error("ad5241 address must be between 44 and 47")
#        scale = config.getfloat('scale', 1., above=0.)
#        val = config.getfloat('wiper', None, minval=0., maxval=scale)
#        out1 = config.getint('out1', None, minval=0, maxval=1)
#        out2 = config.getint('out2', None, minval=0, maxval=1)
#        self.outputs = {'O1':out1, 'O2':out2} # evtl. Liste statt dict
#        # Configure initial state
#        if any(val, out1, out2):
#            out1 = out1 if out1 else 0 # 'is not None'
#            out2 = out2 if out2 else 0
#            val = int(val * 255. / scale) if val else 127 # int(val * 255. / scale + .5)
#            self._set_all(val, out1, out2)
# 
#    def set_wiper_int(self, value):
#        self.i2c.i2c_write([(self.outputs['O1'] << 4) | (self.outputs['O2'] << 3), value])
# 
#    def set_wiper(self, value):
#        val = max(255, min(0, int(value)*255.))
#        self.set_wiper_int(val)
# 
#    def set_output(self, pin, value, latch=True):
#        v = max(1, min(0, int(value)))
#        self.outputs[pin] = v
#        if latch:
#            self.i2c.i2c_write([(self.outputs['O1'] << 4) | (self.outputs['O2'] << 3)])
# 
#    def _set_all(self, wiper, out1, out2):
#        self.i2c.i2c_write([(out1 << 4) | (out2 << 3), wiper])
#        self.outputs = {'O1':out1, 'O2':out2}
# 
#    def get_mcu(self):
#        return self.i2c.get_mcu()
 
 
def load_config_prefix(config):
    return AD5241(config)
 
 
    """
        class MCU_pwm
    ok    get_mcu
        setup_max_duration
        setup_cycle_time
        setup_start_value
        _build_config (!)
        set_pwm
 
        class MCU_digital_out
    ok    get_mcu
        setup_max_duration
        setup_start_value
        _build_config
        set_digital
 
        class MCU_adc
        get_mcu
        setup_adc_sample
        setup_adc_callback
        get_last_value
        _build_config
        _handle_analog_in_state
 
        class MCU_ADS1100
        get_mcu
        setup_minmax
        setup_adc_callback
        get_last_value
        _build_config
        _handle_ready
        _read_response
        _handle_timer
    """