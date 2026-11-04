# DHT11 / DHT22 environmental sensors on auxiliary MCUs.
# This file may be distributed under the terms of the GNU GPLv3 license.
import json

STATUS = ('OK', 'NO_RESPONSE', 'TIMEOUT', 'PULSE_ERROR', 'CHECKSUM',
          'OUT_OF_RANGE', 'BUS_BUSY')
RESULT_FORMAT = ('dht_result oid=%c clock=%u status=%c'
                 ' temp10=%hi humidity10=%hu')


class DHTSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        if not config.get_name().startswith('temperature_sensor '):
            raise config.error('DHT sensors may only be used in '
                               '[temperature_sensor ...], never for heaters')
        if not config.getboolean('dht_auxiliary_mcu', False):
            raise config.error('DHT requires dht_auxiliary_mcu: True; '
                               'check the auxiliary MCU duties in docs/DHT.md')
        self.name = config.get_name().split(' ', 1)[1]
        self.sensor_type = {'DHT11': 11, 'DHT22': 22}[config.get('sensor_type')]
        self.report_time = config.getfloat(
            'dht_report_time', 5., above=1. if self.sensor_type == 11 else 2.,
            maxval=120.)
        self.max_age = config.getfloat('dht_max_age', 15.,
                                       above=self.report_time)
        ppins = self.printer.lookup_object('pins')
        pin = ppins.lookup_pin(config.get('sensor_pin'), can_pullup=True)
        if pin['pullup'] < 0:
            raise config.error('DHT does not support a pull-down')
        self._mcu = pin['chip']
        for name, sensor in self.printer.lookup_objects('dht_status'):
            if sensor.get_mcu() is self._mcu:
                raise config.error('DHT supports only one sensor per MCU')
        self._pin, self._pullup = pin['pin'], pin['pullup']
        self._oid = self._mcu.create_oid()
        self._callback = None
        self.min_temp, self.max_temp = -273.15, 99999999.9
        self._temperature = self._humidity = self._last_read_time = None
        self._last_clock = None
        self.last_error = 'NOT_READY'
        self.ok_count = self.checksum_errors = self.timeout_errors = 0
        self.protocol_errors = self.range_errors = 0
        self.printer.add_object('dht_status ' + self.name, self)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('QUERY_DHT', 'SENSOR', self.name,
                                   self.cmd_QUERY_DHT,
                                   desc=self.cmd_QUERY_DHT_help)
        self._mcu.register_config_callback(self._build_config)
        self._mcu.register_serial_response(self._handle_result,
                                           RESULT_FORMAT, self._oid)

    def get_mcu(self):
        return self._mcu

    def _build_config(self):
        constants = self._mcu.get_constants()
        if (constants.get('DHT_VERSION') != 1
            or constants.get('MCU') != 'atmega328p'
            or constants.get('CLOCK_FREQ') != 16000000):
            raise self.printer.config_error(
                'DHT requires firmware with CONFIG_WANT_DHT on ATmega328P '
                'at 16MHz, without steppers or hardware PWM')
        self._mcu.add_config_cmd(
            'config_dht oid=%d pin=%s sensor_type=%d pull_up=%d'
            % (self._oid, self._pin, self.sensor_type, self._pullup))
        # The firmware also enforces warm-up and minimum start-to-start time.
        clock = self._mcu.get_query_slot(self._oid)
        clock += self._mcu.seconds_to_clock(2.)
        rest = self._mcu.seconds_to_clock(self.report_time)
        self._mcu.add_config_cmd(
            'query_dht oid=%d clock=%d rest_ticks=%d'
            % (self._oid, clock, rest), is_init=True)
        self._mcu.add_config_cmd(
            'query_dht oid=%d clock=0 rest_ticks=0' % (self._oid,),
            on_restart=True)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp, self.max_temp = min_temp, max_temp

    def setup_callback(self, callback):
        self._callback = callback

    def get_report_time_delta(self):
        return self.report_time

    def _handle_result(self, params):
        clock = self._mcu.clock32_to_clock64(params['clock'])
        if self._last_clock is not None and clock <= self._last_clock:
            return
        self._last_clock = clock
        status = params['status']
        if status == 0:
            temp, humidity = params['temp10'] / 10., params['humidity10'] / 10.
            sensor_min, sensor_max = (0., 50.) if self.sensor_type == 11 \
                else (-40., 80.)
            humidity_min, humidity_max = (20., 90.) if self.sensor_type == 11 \
                else (0., 100.)
            if (not max(sensor_min, self.min_temp) <= temp
                    <= min(sensor_max, self.max_temp)
                or not humidity_min <= humidity <= humidity_max):
                status = 5
        self.last_error = STATUS[status] if 0 <= status < len(STATUS) \
            else 'UNKNOWN_STATUS'
        if status:
            if status == 4:
                self.checksum_errors += 1
            elif status in (1, 2):
                self.timeout_errors += 1
            elif status == 5:
                self.range_errors += 1
            else:
                self.protocol_errors += 1
            return
        self.ok_count += 1
        read_time = self._mcu.clock_to_print_time(clock)
        self._temperature, self._humidity = temp, humidity
        self._last_read_time = read_time
        if self._callback is not None:
            self._callback(read_time, temp)

    def get_status(self, eventtime):
        age = None
        if self._last_read_time is not None:
            age = max(0., self._mcu.estimated_print_time(eventtime)
                      - self._last_read_time)
        valid = age is not None and age < self.max_age
        return {'temperature': self._temperature if valid else None,
                'humidity': self._humidity if valid else None,
                'valid': valid, 'last_error': self.last_error,
                'age_seconds': age, 'ok_count': self.ok_count,
                'checksum_errors': self.checksum_errors,
                'timeout_errors': self.timeout_errors,
                'protocol_errors': self.protocol_errors,
                'range_errors': self.range_errors}

    cmd_QUERY_DHT_help = 'Report DHT values, validity, age and error counters'

    def cmd_QUERY_DHT(self, gcmd):
        eventtime = self.printer.get_reactor().monotonic()
        status = self.get_status(eventtime)
        gcmd.respond_info('DHT %s: %s' % (
            self.name, json.dumps(status, sort_keys=True)))


def load_config(config):
    pheaters = config.get_printer().load_object(config, 'heaters')
    pheaters.add_sensor_factory('DHT11', DHTSensor)
    pheaters.add_sensor_factory('DHT22', DHTSensor)
