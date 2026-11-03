# Fehlertolerantes I2C am Laser-Due

## Entwurf und Grenzen

Der Laser verwendet einen Arduino Due (SAM3X8E, native USB-Verbindung).
Der ADS1115 `ads1x1x ardu_ADS1115` am Bus `twi1` liefert die nichtkritischen
Temperaturanzeigen von Seitenkanalverdichter, Pumpe und Y-Stepper.
Für diesen Client wird `i2c_error_tolerant: True` aktiviert. Der Standard
bleibt `False`; andere Clients und MCU-interne Sensor-/Probe-Treiber behalten
ihre bisherige Fehlerbehandlung. Die Option ist derzeit nur im ADS1x1x-Treiber
und mit der unten beschriebenen SAM3X-Hardware-I2C-Firmware unterstützt.

Die Umsetzung besteht aus vier Teilen:

1. **Begrenzte MCU-Arbeit:** Jede Transaktion hat ein gemeinsames Zeitlimit
   von 5 ms für TXRDY, RXRDY und TXCOMP einschließlich STOP. NACK und Timeout
   werden als Fehlerstatus zurückgegeben. Es gibt keine erneute Übertragung
   innerhalb des MCU-Treibers. Fehlgeschlagene Reads liefern keine Nutzdaten;
   teilweise empfangene Bytes gelangen nicht zum Host.
2. **Bus-Recovery:** Der TWI-Controller wird zurückgesetzt und der Takt des
   jeweiligen Clients wiederhergestellt. Mit `CONFIG_ATSAM_I2C_RECOVERY=y`
   folgt außerdem ein Bus-Clear mit bis zu neun SCL-Pulsen und STOP. GPIOs
   treiben ausschließlich Low; High entsteht durch Freigabe der Leitung.
   Das Warten auf eine freigegebene SCL-Leitung ist auf ein gemeinsames
   Recovery-Budget von 200 µs begrenzt. Hinzu kommen wenige Registerzugriffe
   und die abschließende Freigabe der Pins. Diese Recovery setzt einen Bus
   mit einem Master voraus. Ein dauerhaft elektrisch festgehaltenes SCL/SDA
   kann Software nicht reparieren; spätere Abfragen versuchen es erneut.
3. **Client-Policy und Lastbegrenzung:** Ein ausdrücklich toleranter Client
   erhält bei Fehlern eine `I2CError`-Exception. Nach einem Fehler werden
   weitere Zugriffe dieses Clients für eine Sekunde lokal unterdrückt.
   Danach versucht der normale Messzyklus eine neue Konversion. Dadurch
   verursacht ein dauerhafter Defekt am Laser höchstens ungefähr eine
   fehlgeschlagene MCU-Transaktion pro Sekunde. Transportfehler und fehlende
   MCU-Antworten behalten die normale Klipper-Behandlung; sie sind keine
   tolerierten I2C-Fehler.
4. **Diagnose auf dem Host:** Fehler werden je konfiguriertem I2C-Client
   gezählt. Die Statistik verwendet monotone Zeit, maximal 3600
   Sekunden-Buckets je Client und fortlaufende Summen. Ein normaler
   Statusabruf muss die gespeicherte Stunde nicht erneut durchsuchen.
   Der erste Fehler erscheint unmittelbar im Log und auf der G-Code-Konsole
   (auch in Mainsail/Fluidd). Weitere automatische Meldungen sind auf eine
   je Minute und Client begrenzt, während jeder gemeldete Fehler gezählt wird.
   Es gibt keine zusätzlichen
   Statistik-Timer oder Diagnoseabfragen an die MCU.

Das I2C-Polling und die Recovery laufen im Task-Kontext. Sie fügen keine
Interruptsperre hinzu; die bestehenden kurzen GPIO-Registerzugriffe verwenden
weiterhin ihre üblichen Sperren. Stepper-, PWM- und Sicherheits-Timer können
die I2C-Arbeit unterbrechen. Die Wartebudgets begrenzen die Verzögerung anderer
Tasks; bei Interruptlast kann die beobachtete Gesamtzeit entsprechend länger
ausfallen. Eine Messung der tatsächlichen Step-/PWM-Latenzen am Laser ist
zusätzlich zu den automatisierten Tests erforderlich, um das konkrete
elektrische Fehlerbild unter Last zu prüfen.

Ein I2C-Ausfall erzeugt beim ADS1115 weder einen Nullwert noch eine scheinbar
frische Wiederholung des letzten Werts. Der Mess-Callback entfällt und der
letzte erfolgreiche Wert bleibt erhalten. Vorherige echte Bereichsverletzungen
werden dabei nicht zurückgesetzt. Gültige Messwerte außerhalb des zulässigen
Bereichs und ein ADC, der trotz erfolgreicher Kommunikation nicht fertig wird,
behalten den bisherigen Schutzmechanismus. Der tolerante Modus ist für
Anzeigen/Monitoring bestimmt, deren fehlende Werte den Betrieb nicht absichern.

Die Registersequenzen richten sich nach dem
[SAM3X/SAM3A-Datenblatt, Kapitel 33](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf).

## Bauen und Konfigurieren

Im Klipper-Verzeichnis unter Linux mit der üblichen Klipper-ARM-Toolchain:

```sh
cp mcu-build-config-laser-due .config
make olddefconfig
make -j2
```

Das Ergebnis ist `out/klipper.bin`. Die Konfiguration wählt SAM3X8E,
84 MHz, native USB-Kommunikation, USB-Seriennummer aus der Chip-ID und
I2C-Recovery. Die USB-Identität entspricht der bestehenden Laser-Konfiguration.

Klippy und Due-Firmware müssen aus diesem geänderten Stand stammen.
Beim Aktivieren der Toleranz prüft Klippy die Firmware-Konstante
`I2C_BUS_RECOVERY`; eine alte oder ohne Recovery gebaute Firmware wird
mit einer konkreten Fehlermeldung abgewiesen. Auch Software-I2C wird für
diese Option abgewiesen.

### Vergleich mit den vorhandenen Build-Configs

Geprüft wurden `arduino-due.config`, `arduino-due-serial.config` und
`arduino-due-sam3x8e.config` aus `D:/Downloads/config-laser`. Alle drei wählen
den SAM3X8E mit 84 MHz. Die beiden USB-Dateien unterscheiden sich nur durch
`CONFIG_WANT_BMI160=y` in `arduino-due-sam3x8e.config` und Leerzeilen.
Da BMI160 in diesem Quellstand standardmäßig aktiviert ist, ergeben sie nach
Kconfig-Auswertung exakt dieselben Einstellungen. Auch die neue
`mcu-build-config-laser-due` ergibt dieselben Einstellungen, ergänzt um
`CONFIG_ATSAM_I2C_RECOVERY=y`.

`arduino-due-serial.config` wählt stattdessen UART mit 250000 Baud und
deaktiviert natives USB. Die hinterlegte Laser-Verbindung
`/dev/serial/by-id/usb-Klipper_sam3x8e_...-if00` passt zu den USB-Configs.
Die serielle Datei passt somit nicht zu dieser Verbindung. Welche der beiden
gleichwertigen USB-Dateien beim letzten Build verwendet wurde, lässt sich
aus diesen Dateien nicht eindeutig ermitteln. Der Dateiname wird in der
Firmware nicht gespeichert. Eine Live-Abfrage bzw. ein Log der laufenden MCU
wurde für diesen Vergleich nicht herangezogen.

Die Laser-Konfiguration enthält im Abschnitt des vorhandenen ADCs:

```ini
[ads1x1x ardu_ADS1115]
chip: ADS1115
pga: 4.096V
adc_voltage: 3.3
i2c_bus: twi1
i2c_speed: 100000
i2c_error_tolerant: True
```

Es wurde keine laufende Maschine aktualisiert oder geflasht.

## Diagnose

Automatische Konsolenmeldungen enthalten Clientname, MCU, Bus, OID, Adresse,
Fehlerart und Gesamtzahl. Sie halten den Controller nicht an. Die Begrenzung
auf eine Meldung pro Minute und Client verhindert eine Konsolenflut bei einem
dauerhaften Defekt; `I2C_ERRORS` zeigt jederzeit alle gezählten Fehler.

`I2C_ERRORS` zeigt die Fehler der letzten Stunde, die Gesamtzahl seit dem
Klippy-Start sowie Clientname, MCU, Bus, OID, Adresse, Fehlerarten und die
Anzahl lokal unterdrückter Zugriffe. Ein fehlgeschlagener Transfer zählt
einmal, unabhängig von der Zahl betroffener ADC-Kanäle. Ein wegen der
Wartepause unterdrückter Zugriff zählt nicht als weiterer Busfehler.

Die automatisch verfügbare Status-Objektstruktur `i2c_stats` enthält:

```text
errors_last_hour, errors_total, window_seconds, resolution_seconds
clients["mcu:OID"]:
  name, mcu, bus, oid, address
  errors_last_hour, errors_total, by_status_last_hour
  last_error, last_error_time, skipped_total
```

Das gleitende Stundenfenster hat eine Auflösung von einer Sekunde; ein Fehler
kann daher bis zu einer Sekunde vor seinem exakten Alter von 3600 s verfallen.
`last_error_time` ist Host-Monotonzeit, keine Uhrzeit. Ein Klippy-Neustart setzt
die Statistik zurück. Erfasst werden die an Klippy zurückgemeldeten Fehler
der Transfer-Schnittstelle und asynchron bestätigter Writes. Alte Firmware
ohne Fehlerstatus und autonome MCU-Sensorabfragen liefern hierfür keine
Fehlerereignisse.

### Geltungsbereich der Fehlertoleranz

Der SAM-Bustreiber in `src/atsam/i2c.c` enthält keine ADS-spezifische Logik:
Alle Hardware-I2C-Reads und -Writes verwenden dieselben begrenzten Wartezeiten,
Fehlerstatus und Recovery-Pfade, unabhängig von Adresse und Sensortyp.
Dies bedeutet noch keine uneingeschränkte Fehlertoleranz aller Clients.
Normale synchrone Host-Clients lösen weiterhin einen Host-Shutdown aus, wenn
sie einen Fehlerstatus erhalten. Im Host ist das gezielte Überspringen
fehlgeschlagener Operationen bislang für den ADS1x1x implementiert.
Die MCU-internen Treiber für BMI160, LIS2DW, MPU9250 und ICM20948 rufen bei
I2C-Fehlern außerdem weiterhin `i2c_shutdown_on_err()` auf. Diese nachgelagerte
Shutdown-Behandlung wird durch die Recovery-Build-Option nicht deaktiviert.
Der LDC1612 behandelt fehlgeschlagene Proben gesondert und meldet Fehler an
seine Triggerlogik. Ein bloßes Entfernen dieser Prüfungen wäre falsch, weil
die aufrufenden Treiber dann unvollständige Daten als Messwerte verarbeiten
könnten. Die Aussage "kein I2C-Fehler kann irgendwo einen Shutdown auslösen"
ist für diesen Stand daher nicht zutreffend.

Im Status `ads1x1x ardu_ADS1115` zeigt
`channels.AIN1.last_sample_age` das Alter der letzten erfolgreichen Probe in
Sekunden, entsprechend auch für AIN2/AIN3. Vor der ersten Probe ist es `None`.
`invalid_count` bezeichnet echte ungültige Messungen, nicht tolerierte
I2C-Ausfälle. Diese Felder lassen sich über die übliche Moonraker-Objektabfrage
oder in Klipper-Makros auslesen, zum Beispiel:

```jinja2
printer.i2c_stats.errors_last_hour
printer['ads1x1x ardu_ADS1115'].channels.AIN1.last_sample_age
```

## Verifikation

```sh
python3 -m unittest test.test_i2c_stats test.test_i2c_bus_faults \
    test.test_ads1x1x_i2c_faults test.test_atsam_i2c -v
```

Die C-Tests kompilieren den tatsächlichen Treiber und die Transfer-Kommandos
gegen ein deterministisches Register-/Timer-/GPIO-Modell. Ein nativer
C-Compiler wird benötigt; `HOST_CC` kann ihn auswählen. Geprüft werden
NACKs, festhängende TX/RX/STOP-Phasen, ein gemeinsames Zeitlimit auch über
Timer-Überläufe, Recovery und anschließende erfolgreiche Kommunikation,
Open-Drain-Verhalten, festgehaltenes SCL, Client-Takte und leere Fehlerantworten.
Die Host-Tests prüfen Fensterablauf, Fehlerstürme, getrennte Clients,
Logbegrenzung, Pausen vor erneuten Versuchen, verworfene Messungen,
Wiederaufnahme und erhaltene Bereichsprüfungen.

Zur Abnahme am Laser den ADC unter laufender Bewegung gezielt kurz
unterbrechen, `I2C_ERRORS` und `last_sample_age` beobachten und nach dem
Wiederanschließen frische Messwerte prüfen. Dabei die kritischen Ausgänge
und Step-Signale messen. Anschließend einen längeren Busausfall prüfen:
Der Fehlerzähler muss weiter steigen, die Abfragerate begrenzt bleiben und
die Kommunikation nach Beseitigung der Störung selbstständig fortfahren.
