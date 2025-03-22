# Support for Linux OS controlled fans via lm-sensors
#
# Copyright (C) 2025  Timo Hilbig <timo@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
 
import os
import subprocess
import re
import logging

HOST_REPORT_TIME = 5.0
FANCONTROL_CONFIG = "/etc/fancontrol"
KLIPPY_FC_DUMMY = "/tmp/klippy_fancontrol_dummy_temp"
SCRIPTPATH = os.path.expanduser("~") + "/klipper/scripts/"
KLIPPY_FANCONTROL_CONFIG = SCRIPTPATH + "fancontrol_klippy"
FANCONTROL_SWITCH = SCRIPTPATH + "switch_fancontrol.sh"
FAN_CALIBRATION_POINTS = SCRIPTPATH + "fan_calib_points.txt"


# Setup pwm object
#        ppins = self.printer.lookup_object('pins')
#        self.mcu_fan = ppins.setup_pin('pwm', config.get('pin'))
# Setup sensor object
#        pheaters = self.printer.load_object(config, 'heaters')
#        self.sensor = pheaters.setup_sensor(config)
#        self.sensor.setup_minmax(self.min_temp, self.max_temp)
#        self.sensor.setup_callback(self.temperature_callback)
#        pheaters.register_sensor(config, self)
 
class LinuxFan:
    def __init__(self, config):
        self.name = config.get_name()
        fanconffile = config.get('fanconfig', FANCONTROL_CONFIG)
        target_fan = config.get('target_fan', None)
        interpolator = config.get('interpolator', None)
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.stepper_names = config.getlist("stepper", None)
        self.stepper_enable = self.printer.load_object(config, 'stepper_enable')
        self.idle_timeout = config.getint("idle_timeout", default=30, minval=0)
        self.last_on = self.idle_timeout
        self.last_speed = 0.
        self.linearize = FanLinearization()
        self.fan = Fancontrol(fanconffile, target_fan)
        control_conf = {'min_temp': self.fan.min_temp, 
                        'max_temp': self.fan.max_temp, 
                        'max_power': self.linearize.max_power_lin, 
                        'off_below': self.linearize.off_below_lin,  
                        'interpolator': interpolator, 
                        'set_function': self._apply_speed}
        self.control = ControlInterpolation(control_conf)

    def get_mcu(self):
        return None
    
    def _handle_ready(self):
        all_steppers = self.stepper_enable.get_steppers()
        if self.stepper_names is None:
            self.stepper_names = all_steppers
        cmd = [FANCONTROL_SWITCH, 'start', KLIPPY_FANCONTROL_CONFIG]
        subprocess.run(cmd, check=True)
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.callback, reactor.monotonic()+HOST_REPORT_TIME)

    def _handle_shutdown(self):
        cmd = [FANCONTROL_SWITCH, 'stop']
        subprocess.run(cmd, check=True)
 
    def _apply_speed(self, print_time, value):
        value = self.linearize.by_speed(value)
        self.fan.set_fake_temp(value)
 
    def set_speed(self, value, print_time=None):
        value = self.linearize.by_speed(value)
        self.fan.set_fake_temp(value)

    def set_speed_from_command(self, value):
        self.set_speed(value)

    def set_speed_raw(self, value):
        self.fan.set_fake_temp(value)

    def _handle_request_restart(self, print_time):
        pass
 
    def get_status(self, eventtime):
        return self.fan.get_status()
 
    def callback(self, eventtime):
        self.fan.refresh_status()
        # speed = 0.
        speed = self.control.interpolate(self.fan.temp)
        active = False
        for name in self.stepper_names:
            active |= self.stepper_enable.lookup_enable(name).is_motor_enabled()
        if active:
            self.last_on = 0
            # speed = self.fan_speed
            # Temperatur-Interpolation + Booster durch aktive Stepper
            speed = (speed + self.fan.max_power) / 2.
        elif self.last_on < self.idle_timeout:
            # speed = self.idle_speed
            # Nur Temperatur-Interpolation
            self.last_on += 1
        if speed != self.last_speed:
            self.last_speed = speed
            self.set_speed(speed)
        return eventtime + HOST_REPORT_TIME
    

class Fancontrol:
    def __init__(self, configfile, target):
        config = self.read_config(configfile)
        target = target if target is not None and target in config else 'fan1'
        self.pwm_file = config[target]['abs_pwm']
        self.temp_file = config[target]['abs_temp']
        self.tacho_file = config[target]['abs_fan']
        t = int(self.get_temp() * 1000)
        with open(KLIPPY_FC_DUMMY, "a") as f: f.write(f"{t}\n")
        self.fake_temp = KLIPPY_FC_DUMMY
        config[target]['temp'] = self.fake_temp
        self.write_config(config, KLIPPY_FANCONTROL_CONFIG)
        self.pwm = self.temp = self.rpm = 0
        self.min_temp = float(config[target]['MINTEMP'])
        self.max_temp = float(config[target]['MAXTEMP'])
        self.min_pwm = float(config[target]['MINPWM'])
        self.max_pwm = float(config[target]['MAXPWM'])
        self.min_start = float(config[target]['MINSTART'])
        self.min_stop = float(config[target]['MINSTOP'])

        self.off_below = float(self.min_stop/255.) # 0.24
        self.max_power = float(self.max_pwm/255.)

        linear_b = (self.off_below*self.max_temp - self.max_power*self.min_temp) / (self.off_below - self.max_power)
        linear_m = (self.max_temp - linear_b) / self.max_power
        self.linear_b = 1000. * linear_b
        self.linear_m = 1000. * linear_m        
    
    def _get_sym_path(self, rel_path:str):
        if rel_path.startswith("/"):
            return rel_path
        return os.path.join("/sys/class/hwmon", rel_path)

    def _get_abs_path(self, rel_path:str, devpath_map):
        if rel_path.startswith("/"):
            return rel_path
        parts = rel_path.split('/', 1)
        if len(parts) != 2:
            return rel_path
        device_key, _ = parts
        if device_key in devpath_map:
            dp = devpath_map[device_key]
            if dp.startswith("devices/"):
                dp = dp[len("devices/"):]
            return os.path.join("/sys/devices", dp, "hwmon", rel_path)
        else:
            return self._get_sym_path(rel_path)

    def _parse_mapping(self, raw_value):
        mapping = {}
        for token in raw_value.split():
            if "=" in token:
                key, val = token.split("=", 1)
                mapping[key.strip()] = val.strip()
        return mapping

    def read_config(self, config_file):
        raw_config = {}
        with open(config_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    key, val = line.split("=", 1)
                    raw_config[key.strip()] = val.strip()
        
        # Allgemeine Einstellungen
        config_out = {}
        general = {}
        # INTERVAL
        if "INTERVAL" in raw_config:
            try:
                general["INTERVAL"] = int(raw_config["INTERVAL"])
            except ValueError:
                raise ValueError("INTERVAL muss eine ganze Zahl sein.")
        else:
            general["INTERVAL"] = 10  # Default
        
        # DEVPATH und DEVNAMES (im Output unter DEVNAMES)
        if "DEVPATH" in raw_config:
            general["DEVPATH"] = self._parse_mapping(raw_config["DEVPATH"])
        else:
            general["DEVPATH"] = {}
        if "DEVNAME" in raw_config:
            general["DEVNAMES"] = self._parse_mapping(raw_config["DEVNAME"])
        else:
            general["DEVNAMES"] = {}
        
        config_out["general"] = general
        devpath_map = general["DEVPATH"]
        
        # Zuordnung der Fanparameter erfolgt über den "pwm" Schlüssel.
        # FCTEMPS: Mapping von pwm -> temp
        if "FCTEMPS" not in raw_config:
            raise ValueError("FCTEMPS fehlt in der Konfigurationsdatei.")
        fctemps_map = self._parse_mapping(raw_config["FCTEMPS"])
        
        # FCFANS: Mapping von pwm -> fan (ggf. mit führenden Leerzeichen)
        fcfans_map = {}
        if "FCFANS" in raw_config:
            fcfans_map = self._parse_mapping(raw_config["FCFANS"])
        
        # Numerische Parameter (MINTEMP, MAXTEMP, MINSTART, MINSTOP, MINPWM, MAXPWM, AVERAGE)
        def get_numeric_mapping(key, default=0):
            if key in raw_config:
                m = self._parse_mapping(raw_config[key])
                return {k: int(v) for k, v in m.items()}
            else:
                return {}
        
        mintemp_map  = get_numeric_mapping("MINTEMP", default=0)
        maxtemp_map  = get_numeric_mapping("MAXTEMP", default=100)
        minstart_map = get_numeric_mapping("MINSTART", default=0)
        minstop_map  = get_numeric_mapping("MINSTOP", default=0)
        minpwm_map   = get_numeric_mapping("MINPWM", default=0)
        maxpwm_map   = get_numeric_mapping("MAXPWM", default=255)
        average_map  = get_numeric_mapping("AVERAGE", default=1)
        
        # Erstelle für jeden Fan einen eigenen Eintrag.
        fan_index = 1
        for pwm_key, temp_val in fctemps_map.items():
            entry = {}
            entry["pwm"]  = pwm_key
            entry["temp"] = temp_val
            # FCFANS-Zuordnung (falls vorhanden)
            entry["fan"] = fcfans_map.get(pwm_key, "")
            
            # Symbolische Pfade (z. B. unter /sys/class/hwmon)
            entry["sym_pwm"]  = self._get_sym_path(pwm_key)
            entry["sym_temp"] = self._get_sym_path(temp_val)
            entry["sym_fan"]  = self._get_sym_path(entry["fan"]) if entry["fan"] else ""
            
            # Absolute Pfade (auf Basis von DEVPATH)
            entry["abs_pwm"]  = self._get_abs_path(pwm_key, devpath_map)
            entry["abs_temp"] = self._get_abs_path(temp_val, devpath_map)
            entry["abs_fan"]  = self._get_abs_path(entry["fan"], devpath_map) if entry["fan"] else ""
            
            # Numerische Werte; falls für einen pwm-Eintrag kein Wert vorhanden ist, wird ein Dedaultwert übernommen.
            entry["MINTEMP"]  = mintemp_map.get(pwm_key, 0)
            entry["MAXTEMP"]  = maxtemp_map.get(pwm_key, 100)
            entry["MINSTART"] = minstart_map.get(pwm_key, 0)
            entry["MINSTOP"]  = minstop_map.get(pwm_key, 0)
            entry["MINPWM"]   = minpwm_map.get(pwm_key, entry["MINSTOP"])
            entry["MAXPWM"]   = maxpwm_map.get(pwm_key, 255)
            entry["AVERAGE"]  = average_map.get(pwm_key, 1)
            
            # Schlüssel: fan1, fan2, …
            fan_key = f"fan{fan_index}"
            config_out[fan_key] = entry
            fan_index += 1

        return config_out

    def write_config(self, config, filename):
        lines = []
        # Header-Kommentar
        lines.append("# Configuration file generated by klippy_fancontrol config writer, manual changes will be lost")
        
        # Allgemeine Einstellungen
        general = config.get("general", {})
        interval = general.get("INTERVAL", 10)
        lines.append(f"INTERVAL={interval}")
        
        # DEVPATH: Die Zuordnung erfolgt als key=value, getrennt durch Leerzeichen
        devpath = general.get("DEVPATH", {})
        devpath_line = "DEVPATH=" + " ".join(f"{k}={v}" for k, v in devpath.items())
        lines.append(devpath_line)
        
        # DEVNAME: In der Konfigurationsdatei heißt der Schlüssel DEVNAME (nicht DEVNAMES)
        devnames = general.get("DEVNAMES", {})
        devname_line = "DEVNAME=" + " ".join(f"{k}={v}" for k, v in devnames.items())
        lines.append(devname_line)
        
        # Listen zur Sammlung der Fan-bezogenen Parameter
        fctemps_tokens = []
        fcfans_tokens = []
        mintemp_tokens = []
        maxtemp_tokens = []
        minstart_tokens = []
        minstop_tokens = []
        minpwm_tokens = []
        maxpwm_tokens = []
        average_tokens = []
        
        # Alle Keys, die nicht "general" sind, gelten als Fan-Einträge.
        fan_keys = [key for key in config if key != "general"]
        fan_keys.sort()  # für konsistente Ausgabe
        for fan_key in fan_keys:
            fan_entry = config[fan_key]
            pwm = fan_entry.get("pwm", "").strip()
            temp = fan_entry.get("temp", "").strip()
            fan_val = fan_entry.get("fan", "").strip()
            if pwm:
                if temp:
                    fctemps_tokens.append(f"{pwm}={temp}")
                if fan_val:
                    fcfans_tokens.append(f"{pwm}={fan_val}")
                # Numerische Parameter: Falls der Eintrag vorhanden ist, wird er übernommen.
                if "MINTEMP" in fan_entry:
                    mintemp_tokens.append(f"{pwm}={fan_entry['MINTEMP']}")
                if "MAXTEMP" in fan_entry:
                    maxtemp_tokens.append(f"{pwm}={fan_entry['MAXTEMP']}")
                if "MINSTART" in fan_entry:
                    minstart_tokens.append(f"{pwm}={fan_entry['MINSTART']}")
                if "MINSTOP" in fan_entry:
                    minstop_tokens.append(f"{pwm}={fan_entry['MINSTOP']}")
                if "MINPWM" in fan_entry:
                    minpwm_tokens.append(f"{pwm}={fan_entry['MINPWM']}")
                if "MAXPWM" in fan_entry:
                    maxpwm_tokens.append(f"{pwm}={fan_entry['MAXPWM']}")
                if "AVERAGE" in fan_entry:
                    average_tokens.append(f"{pwm}={fan_entry['AVERAGE']}")
        
        # Falls entsprechende Einträge vorhanden sind, werden die Zeilen erzeugt.
        if fctemps_tokens:
            lines.append("FCTEMPS=" + " ".join(fctemps_tokens))
        if fcfans_tokens:
            lines.append("FCFANS=" + " ".join(fcfans_tokens))
        if mintemp_tokens:
            lines.append("MINTEMP=" + " ".join(mintemp_tokens))
        if maxtemp_tokens:
            lines.append("MAXTEMP=" + " ".join(maxtemp_tokens))
        if minstart_tokens:
            lines.append("MINSTART=" + " ".join(minstart_tokens))
        if minstop_tokens:
            lines.append("MINSTOP=" + " ".join(minstop_tokens))
        if minpwm_tokens:
            lines.append("MINPWM=" + " ".join(minpwm_tokens))
        if maxpwm_tokens:
            lines.append("MAXPWM=" + " ".join(maxpwm_tokens))
        if average_tokens:
            lines.append("AVERAGE=" + " ".join(average_tokens))
        
        # Schreibe alle Zeilen in die Datei
        with open(filename, "w") as f:
            for line in lines:
                f.write(line + "\n")

    def get_status(self, eventtime=None):
        return {
            "speed": self.pwm/255.,
            "rpm": self.rpm,
            "temperature": self.temp,
            "target": 20.0,
            "display": True,
            "type": "controller_fan"
        }

    def get_temp(self):
        try:
            with open(self.temp_file, "r") as temp_file:
                temp_file.seek(0)
                temp = float(temp_file.read())/1000.0
                self.temp = temp
            return temp
        except:
            logging.exception("could not read linux temperature")
            return -1

    def get_tacho(self):
        try:
            with open(self.tacho_file, "r") as tacho_file:
                tacho_file.seek(0)
                rpm = int(tacho_file.read())
                self.rpm = rpm
            return rpm
        except:
            logging.exception("could not read linux tacho signal")
            return -1

    def get_pwm(self):
        try:
            with open(self.pwm_file, "r") as pwm_file:
                pwm_file.seek(0)
                pwm = int(pwm_file.read())
                self.pwm = pwm
            return pwm
        except:
            logging.exception("could not read linux pwm")
            return -1

    def refresh_status(self, eventtime=None):
        self.get_temp()
        self.get_tacho()
        self.get_pwm()

    def set_fake_temp(self, value, convert=True):
        if convert:
            value = self.convert_speed_to_fake_temp(value)
        checked_value = int(max(min(value, self.max_temp*1000.), self.min_temp*1000.))
        try:
            with open(self.fake_temp, "w") as ft_file:
                ft_file.write(f"{checked_value}\n")
        except:
            logging.exception("could not write linux pwm")
        
    def convert_speed_to_fake_temp(self, speed_value):
        fake_temp_v = self.linear_m * speed_value + self.linear_b
        return fake_temp_v
 

class ControlInterpolation:
    def __init__(self, config):
        self.min_temp = config['min_temp']
        self.max_temp = config['max_temp']
        self.max_power = config['max_power']
        self.off_below = config['off_below']
        self.default_interpolator = config['interpolator'] or 'array'
        self.interpolators = {
            'linear': self.interpolate_lin,
            'quadratic': self.interpolate_quad,
            'exponential': self.interpolate_exp,
            'array': self.interpolate_array
        }
        self._generate_interpolation_coefficients()
        self.set_lf_speed = config.get('set_function', None)

    def add_array_interpolation_point(self, temp, power):
        if temp >= self.max_temp \
            or temp <= self.array_interpol_xs[-2] \
            or power < self.off_below \
            or power > self.max_power:
            return
        last_temp = self.array_interpol_xs.pop()
        last_power = self.array_interpol_ys.pop()
        self.array_interpol_xs.append(temp)
        self.array_interpol_xs.append(last_temp)
        self.array_interpol_ys.append(power)
        self.array_interpol_ys.append(last_power)
        self._refresh_array_interpolation_coefficients()
    
    def _refresh_array_interpolation_coefficients(self):
        self.array_interpol_coeffs = [ 
            (self.array_interpol_ys[i + 1] - self.array_interpol_ys[i]) 
            / (self.array_interpol_xs[i + 1] - self.array_interpol_xs[i]) 
            for i, _ in enumerate(self.array_interpol_xs[:-1]) ]

    def _generate_interpolation_coefficients(self):
        self.lin_interpol_coeff = (self.max_power - self.off_below) / (self.max_temp - self.min_temp)
        self.quad_interpol_coeff = self.lin_interpol_coeff / (self.max_temp - self.min_temp)
        self.exp_interpol_coeff1 = self.max_power / self.off_below
        self.exp_interpol_coeff2 = self.max_temp - self.min_temp
        self.array_interpol_xs = [self.min_temp, self.max_temp]
        self.array_interpol_ys = [self.off_below, self.max_power]
        self._refresh_array_interpolation_coefficients()
 
    def interpolate_lin(self, temp):
        return self.off_below + self.lin_interpol_coeff * (temp - self.min_temp)
 
    def interpolate_quad(self, temp):
        return self.off_below + self.quad_interpol_coeff * (temp - self.min_temp)**2
 
    def interpolate_exp(self, temp):
        return self.off_below * self.exp_interpol_coeff1 ** ((temp - self.min_temp) / self.exp_interpol_coeff2)
 
    def interpolate_array(self, temp):
        position = 0
        while self.array_interpol_xs[position + 1] < temp:
            position += 1
        y0 = self.array_interpol_ys[position]
        x0 = self.array_interpol_xs[position]
        coeff = self.array_interpol_coeffs[position]
        return y0 + coeff * (temp - x0)
 
    def interpolate(self, temp, method=None):
        if temp < self.min_temp:
            return self.off_below
        elif temp > self.max_temp:
            return self.max_power
        if method is None:
            method = self.default_interpolator
        interpolated = self.interpolators[method](temp)
        return interpolated
        # Interpolation: Linear, quadratisch, exponentiell

    def set_default_interpolator(self, interpolator):
        if interpolator not in self.interpolators.keys():
            return
        self.default_interpolator = interpolator

    def temperature_callback(self, read_time, temp):
        if self.set_lf_speed is not None:
            fpower = self.interpolate(temp)
            self.set_lf_speed(read_time, fpower)


class FanLinearization:            
    def __init__(self, calib_points=FAN_CALIBRATION_POINTS):
        separators = ":,;>"
        self.mapping = []
        with open(calib_points, "r") as f:
            for line in f:
                if line.startswith('#'): continue
                separator = next((s for s in separators if s in line), None)
                pwm, rpm = line.split(separator)
                self.mapping.append({'pwm': int(pwm.strip()), 'rpm': int(rpm.strip())})
        for i in self.mapping:
            i['pwr_abs'] = i['pwm'] / 255
            i['pwr_rel'] = i['rpm'] / self.mapping[-1]['rpm']
            i['pwm_lin'] = int(i['pwr_rel'] * self.mapping[-1]['pwm'])
        self.array_interpol_coeffs = [ 
            {'pwr_abs': (self.mapping[i + 1]['pwr_abs'] - self.mapping[i]['pwr_abs']) 
                      / (self.mapping[i + 1]['pwr_rel'] - self.mapping[i]['pwr_rel']),
             'pwm': (self.mapping[i + 1]['pwm']     - self.mapping[i]['pwm']) 
                  / (self.mapping[i + 1]['pwm_lin'] - self.mapping[i]['pwm_lin']) }
            for i, _ in enumerate(self.mapping[:-1]) ]
        self.off_below_lin = self.mapping[0]['pwr_rel']
        self.max_power_lin = self.mapping[-1]['pwr_rel']
        
    def by_speed(self, speed_in): # speed_in -> gewünschte Drehzahl (linear) -> speed_out (gemappt)
        return self._interpolate(speed_in, target='pwr_abs', source='pwr_rel', fmt=float)

    def by_pwm(self, pwm_in): # pwm_in -> gewünschte Drehzahl (linear) -> pwm_out (gemappt)
        return self._interpolate(pwm_in, target='pwm', source='pwm_lin', fmt=int)

    def _interpolate(self, x, target, source, fmt):
        position = 0
        while self.mapping[position + 1][source] < x:
            position += 1
        y0 = self.mapping[position][target]
        x0 = self.mapping[position][source]
        coeff = self.array_interpol_coeffs[position][target]
        return fmt(y0 + coeff * (x - x0))
    
def load_config(config):
    return LinuxFan(config)