# Extension of the load cell probe module to prime the extruder based on the
# measured force.
#
# Copyright (C) 2025  Timo Hilbig <timo@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging, time

class PressurePriming:
    def __init__(self, config):
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        self.force_threshold = config.getint('force_threshold', minval=1.)
        self.force_threshold_default = self.force_threshold
        self.max_prime_length = config.getfloat('max_prime_length', minval=1.)
        self.max_prime_length_default = self.max_prime_length
        self.force_safety_limit = config.getint('force_safety_limit', 8000, 
                                                maxval=10000, minval=1.)

        self.load_cell = self.printer.lookup_object('load_cell')
        #self.load_cell.subscribe_force(self.force_callback)

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('PRESSURE_PRIME',
            self.cmd_PRESSURE_PRIME,
            desc=self.cmd_PRESSURE_PRIME_help)
        
        self.force_protocol = [[]]
        self.mean_force = []
        self.loop = 0
        self.force_delta = []
        self.primed = False
        self.threshold_reached = False
        self.overpressure = False
        self.running = False
 
        # Register transform
        #gcode_move = self.printer.load_object(config, 'gcode_move')
 
    cmd_PRESSURE_PRIME_help = "Prime Extruder by loading Filament " \
        "until force rises above threshold."
   
    def _handle_ready(self):
        self.tool = self.printer.lookup_object('toolhead')
 
    def force_callback(self, force):
        if not self.running:
            return
        if force > self.force_safety_limit:
            self.overpressure = True
        self.force_protocol[self.loop].append(force)
 
    def cmd_PRESSURE_PRIME(self, gcmd):
        extr = self.printer.lookup_object('extruder1') if gcmd.get_int('EXTRUDER', 0) == 1 else self.printer.lookup_object('extruder')
        ttemp = gcmd.get_float('TARGET_TEMP', 210., minval=extr.heater.min_extrude_temp, maxval=extr.heater.max_temp)
        self.force_threshold = gcmd.get_int('THRESHOLD', self.force_threshold_default, minval=1, maxval=self.force_safety_limit)
        f_limit = gcmd.get_int('LIMIT', self.force_safety_limit, minval=1, maxval=self.force_safety_limit)
        self.max_prime_length = gcmd.get_float('LENGTH', self.max_prime_length_default, minval=1, maxval=100)
        speed = gcmd.get_float('SPEED', 45*60, minval=5*60, maxval=150*60) / 60.

        starttime = self.reactor.monotonic()
        gcmd.respond_info(f"Starting Pressure Priming at timestamp {starttime:.0f} - Parameters: \n ttemp: {ttemp:.1f}\n force ts: {self.force_threshold}\n force limit: {f_limit}\n Prime length: {self.max_prime_length}\n speed: {speed}")
        max_priming_dur = self.max_prime_length / (extr.max_e_velocity * speed / self.tool.get_max_velocity()[0]) + 2
        gcmd.respond_info(f"Max. Priming Duration: {max_priming_dur:.2f} s")
        
        # Check if target temp is reached, otherwise wait for it
        status = extr.get_status(starttime)
        if ttemp != status['target']:
            gcmd.respond_info(f"Extruder Target Temp not matching Pressure Priming target temp.\nAdjusting to PP-Target: {ttemp:.0f}")
            pheaters = self.printer.lookup_object('heaters') # einfacher über run_script()!
            pheaters.set_temperature(extr.get_heater(), ttemp, wait=True)
        elif ttemp - status['temperature'] > 2:
            gcmd.respond_info(f"Extruder has not reached target yet ({status['temperature']:.0f}/{ttemp:.0f})")
            counter = 0
            while ttemp - status['temperature'] > 2:
                eventtime = self.reactor.monotonic()
                self.reactor.pause(eventtime + 1.000)
                counter += 1
                gcmd.respond_info(f"Waiting for Extruder to heat up - {counter}s")
                if counter > 180:
                    gcmd.error("Aborting - Extruder not heating up")
                    break
        starttime = self.reactor.monotonic()
        gcmd.respond_info(f"Extruder heated up. Extrusion start time: {starttime:.0f}")
        timeout = starttime + max_priming_dur
        self.gcode.run_script("LCP_COMPENSATE")
        self.load_cell.subscribe_force(self.force_callback)

        while not self.primed:
            self.running = True # Kraftmessung aktivieren
            pos = self.tool.get_position() # Alte Position holen
            pos[3] += 1. # 1 mm Auf Extruderposition addieren
            self.tool.manual_move(pos, speed) # 1 mm extrudieren
            self.tool.wait_moves() # Auf Finish warten
            self.running = False # Kraftmessung deaktivieren
            # Kraftmessung verarbeiten
            measurements = len(self.force_protocol[self.loop]) # Anzahl der Messungen während der Extrusion
            cutoff = max(3, int(measurements/6)) # Ränder ignorieren --> Länge des Randbereichs bestimmen
            force_sum = sum(self.force_protocol[self.loop][cutoff:-cutoff]) if cutoff else sum(self.force_protocol[self.loop]) # Summe für Mittelwert bilden. Nur Blockmitte betrachten
            mean_force =  force_sum / (measurements - 2*cutoff) # Mittelwert berechnen
            delta_f = mean_force - self.mean_force[-1] if self.loop else mean_force # Differenz zu vorheriger Messung bestimmen 
            self.mean_force.append(mean_force) # Mittelwert loggen
            self.force_delta.append(delta_f) # Delta loggen
            mean_force = 0.1 # !!!!!!!!!!!!!!!!!!!!! Dummy -> Quasi-Nullen, um Schleife durchlaufen zu lassen
            self.loop += 1 # loop inkrementieren
            if mean_force > self.force_threshold: # Schwellwert erreicht
                if not self.threshold_reached: # Falls erstes Mal, Flag setzen
                    self.threshold_reached = True
                elif abs(delta_f / mean_force) < 0.15: # Falls nicht das erste Mal: Abweichung bewerten
                    self.primed = True
                    gcmd.respond_info(f"Priming successful! Extruded {self.loop} mm")
            else:
                self.threshold_reached = False # Falls Kraft (wieder) unter Schwellwert liegt, Flag zurücksetzen
            gcmd.respond_info(", ".join(self.force_protocol[self.loop])) # Status ausgeben
            # auf Abbruchkriterien prüfen & ggf. Schleife abbrechen 
            if self.overpressure:
                gcmd.error("Aborting - Force Safety-Limit exceeded") # respond message
                break
            elif self.loop > self.max_prime_length:
                gcmd.error("Aborting - Max Priming Length exceeded") # respond message
                break
            elif self.reactor.monotonic() > timeout:
                gcmd.error("Aborting - Timeout reached") # respond message
                break

        self.running = False
        #self.load_cell._force_callbacks.pop()
        gcmd.respond_info(f"Ending Pressure Priming at timestamp {self.reactor.monotonic()} with status {'SUCCESS.' if self.primed else 'FAILURE!'}")
        summary = [f"Nr {i:>3}: F_m={self.mean_force[i]:4.1f}; F_d={self.force_delta[i]:4.1f}" for i in range(self.loop)]
        gcmd.respond_info("\n".join(summary))


def load_config(config):
    return PressurePriming(config)
