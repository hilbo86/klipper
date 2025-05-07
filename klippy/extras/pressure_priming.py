# Extension of the load cell probe module to prime the extruder based on the
# measured force.
#
# Copyright (C) 2025  Timo Hilbig <timo@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import time
from datetime import datetime

class PressurePriming:
    def __init__(self, config):
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler('klippy:ready', self._handle_ready)

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
        self.reset_ttemp = None

        # Register transform
        #gcode_move = self.printer.load_object(config, 'gcode_move')

    cmd_PRESSURE_PRIME_help = 'Prime Extruder by loading Filament ' \
        'until force rises above threshold.'

    def _handle_ready(self):
        self.tool = self.printer.lookup_object('toolhead')

    def force_callback(self, force):
        if not self.running:
            return
        if force > self.force_safety_limit:
            self.overpressure = True
        self.force_protocol[self.loop].append(force)

    def cmd_PRESSURE_PRIME(self, gcmd):
        extr = self.printer.lookup_object('extruder1') \
            if gcmd.get_int('EXTRUDER', 0) == 1 \
            else self.printer.lookup_object('extruder')
        ttemp = gcmd.get_float('TARGET_TEMP',
                               210.,
                               minval=extr.heater.min_extrude_temp,
                               maxval=extr.heater.max_temp)
        self.force_threshold = gcmd.get_int('THRESHOLD',
                                            self.force_threshold_default,
                                            minval=1,
                                            maxval=self.force_safety_limit)
        f_limit = gcmd.get_int('LIMIT',
                               self.force_safety_limit,
                               minval=1,
                               maxval=self.force_safety_limit)
        self.max_prime_length = gcmd.get_float('LENGTH',
                                               self.max_prime_length_default,
                                               minval=1,
                                               maxval=100)
        speed = gcmd.get_float('SPEED', 2*60, minval=0.5*60, maxval=15*60)/60.

        starttime = self.reactor.monotonic()
        max_priming_dur = self.max_prime_length / min(extr.max_e_velocity, speed) * 2  # Speed 2: 0.806 s/mm
        gcmd.respond_info(
            f'Starting Pressure Priming at timestamp ' \
            f'{starttime:.0f} - Parameters: \n' \
            f'ttemp: {ttemp:.1f}\n' \
            f'force ts: {self.force_threshold}\n' \
            f'force limit: {f_limit}\n' \
            f'Prime length: {self.max_prime_length}\n' \
            f'Extrusion speed: {speed:.2f}\n' \
            f'Max E-Speed: {extr.max_e_velocity:.2f}\n' \
            f'Max. Priming Duration: {max_priming_dur:.2f} s'            
        )

        # Check if target temp is reached, otherwise wait for it
        status = extr.get_status(starttime)
        if ttemp != status['target']:
            self.reset_ttemp = status['target']
            gcmd.respond_info(f'Extruder Target Temp ({status["target"]}) ' \
                              'not matching Pressure Priming target temp.\n' \
                              f'Adjusting to PP-Target: {ttemp:.0f}')
            # folgendes einfacher über run_script()!
            pheaters = self.printer.lookup_object('heaters')
            pheaters.set_temperature(extr.get_heater(), ttemp, wait=True)
        elif ttemp - status['temperature'] > 2:
            gcmd.respond_info(
                'Extruder has not reached target yet ('\
                    f'{status["temperature"]:.0f}/{ttemp:.0f})')
            counter = 0
            while ttemp - status['temperature'] > 2:
                eventtime = self.reactor.monotonic()
                self.reactor.pause(eventtime + 1.000)
                counter += 1
                gcmd.respond_info(
                    f'Waiting for Extruder to heat up - {counter}s')
                if counter > 180:
                    gcmd.error('Aborting - Extruder not heating up')
                    break
        starttime = self.reactor.monotonic()
        gcmd.respond_info(
            f'Extruder heated up. Extrusion start time: {starttime:.0f}')
        timeout = starttime + max_priming_dur
        self.gcode.run_script_from_command('LCP_COMPENSATE\n')
        gcmd.respond_info(
            f'Load Cell Probe calibrated')
        self.load_cell.subscribe_force(self.force_callback)
        gcmd.respond_info(
            f'Registered Force Callback')
        gcmd.respond_info(
            f'processing priming at speed {speed}'
        )

        while not self.primed:
            self.running = True # Kraftmessung aktivieren
            pos = self.tool.get_position() # Alte Position holen
            pos[3] += 1. # 1 mm Auf Extruderposition addieren
            e_start = datetime.now()
            self.tool.manual_move(pos, speed) # 1 mm extrudieren
            self.tool.wait_moves() # Auf Finish warten
            e_end = datetime.now()
            dur = e_end - e_start
            dur = dur.total_seconds()
            gcmd.respond_info(
                f'took {dur:.3f} to extrude 1 mm at {speed}'
            )
            self.running = False # Kraftmessung deaktivieren
            # Kraftmessung verarbeiten
            # Anzahl der Messungen während der Extrusion
            measurements = len(self.force_protocol[self.loop]) if len(self.force_protocol) > self.loop else 0
            cutoff = max(3, int(measurements/6)) # Ränder ignorieren
            # --> Länge des Randbereichs bestimmen
            # Summe für Mittelwert bilden. Nur Blockmitte betrachte
            force_sum = sum(self.force_protocol[self.loop][cutoff:-cutoff]) \
                if cutoff else sum(self.force_protocol[self.loop])
            # Mittelwert berechnen
            mean_force =  force_sum / (measurements - 2*cutoff)
            # Differenz zu vorheriger Messung bestimmen
            delta_f = mean_force - self.mean_force[-1] \
                if self.loop else mean_force
            self.mean_force.append(mean_force) # Mittelwert loggen
            self.force_delta.append(delta_f) # Delta loggen
            #mean_force = 0.1 # !!!!!!!!!!!!!!!!!!!!! Dummy
            # -> Quasi-Nullen, um Schleife durchlaufen zu lassen
            gcmd.respond_info(', '.join([f'{x:.0f}' for x in self.force_protocol[self.loop]]))
            self.loop += 1 # loop inkrementieren
            self.force_protocol.append([])
            self.running = True
            if mean_force > self.force_threshold: # Schwellwert erreicht
                if not self.threshold_reached: # Falls erstes Mal, Flag setzen
                    self.threshold_reached = True
                elif abs(delta_f / mean_force) < 0.15:
                    # Falls nicht das erste Mal: Abweichung bewerten
                    self.primed = True
                    gcmd.respond_info(
                        f'Priming successful! Extruded {self.loop} mm')
            else:
                self.threshold_reached = False
                # Falls Kraft (wieder) unter Schwellwert, Flag zurücksetzen
            # auf Abbruchkriterien prüfen & ggf. Schleife abbrechen
            if self.overpressure:
                gcmd.respond_info('Aborting - Force Safety-Limit exceeded') # error statt respond_info
                break
            elif self.loop > self.max_prime_length:
                gcmd.respond_info('Aborting - Max Priming Length exceeded')
                break
            elif self.reactor.monotonic() > timeout:
                gcmd.respond_info('Aborting - Timeout reached')
                break

        self.running = False
        #self.load_cell._force_callbacks.pop()
        gcmd.respond_info('Ending Pressure Priming at timestamp ' \
                          f'{self.reactor.monotonic():.1f} with status ' \
                          f'{"SUCCESS." if self.primed else "FAILURE!"}')
        summary = [f'Nr {i:>3}: F_mean={self.mean_force[i]:>4.0f}; ' \
                   f'F_d={self.force_delta[i]:>4.0f}' for i in range(self.loop)]
        gcmd.respond_info('\n'.join(summary))
        if self.reset_ttemp is not None:
            gcmd.respond_info(f'Resetting target temperature to {self.reset_ttemp}')
            pheaters = self.printer.lookup_object('heaters')
            pheaters.set_temperature(extr.get_heater(), self.reset_ttemp)


def load_config(config):
    return PressurePriming(config)
