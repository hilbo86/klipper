# DHT11 / DHT22 auf AVR-Nebencontrollern

Stand: 03.11.2026. Implementierung für den lokalen RF2000-Fork von Klipper.
Die Software ist offline geprüft. Eine Hardwareabnahme hat noch nicht
stattgefunden. Keine Firmware wurde geflasht, keine produktive Konfiguration
geändert und kein Dienst neu gestartet.

## Voraussetzungen

- ATmega328P mit tatsächlich 16 MHz, ein Sensor je MCU.
- DATA auf PB5 / Arduino D13 über PCINT5 oder PD2 / Arduino D2 über INT0.
- Nebencontroller ohne Stepper und ohne andere harte Echtzeitaufgaben.
- Externer Pull-up und gemeinsame Masse; Pegel am realen Board prüfen.
- Ausschließlich Umgebungsdaten in `[temperature_sensor ...]`.

Die Buildoption `CONFIG_WANT_DHT` ist standardmäßig aus. Sie lässt sich nur
auf ATmega328P/16 MHz ohne `CONFIG_HAVE_STEPPER` und ohne
`CONFIG_WANT_HARD_PWM` aktivieren. Das verhindert auch eine Belegung der
Timer1-Vergleichsregister durch Hardware-PWM. Andere AVR-Varianten,
insbesondere ATmega328 ohne P oder LGT8F328P, sind nicht freigegeben.
Die Auswahl im Build ersetzt keine Prüfung der Chipbeschriftung und des Takts.

`dht_auxiliary_mcu: True` bestätigt die manuelle Prüfung der MCU-Aufgaben.
Insbesondere Neopixel, zeitkritische Softwareprotokolle, schnelle Software-PWM
und andere Interruptsperren sind vor Einsatz zu prüfen. Der Treiber kann
deren Laufzeit nicht aus einem Sensorpin ableiten. Eine MCU mit DHT darf
nicht später um Stepper oder andere harte Echtzeitaufgaben erweitert werden.

## Konfiguration und Status

Vollständiges Muster: [sample-dht.cfg](../config/sample-dht.cfg).
Die MCU-Namen im Muster sind Platzhalter. Die lokale RF2000-Konfiguration
legt folgende Zuordnung nahe, die am Gerät noch bestätigt werden muss:

```ini
[temperature_sensor waermetauscher]
sensor_type: DHT11
sensor_pin: wtrbrd:PB5
dht_auxiliary_mcu: True
dht_report_time: 5.0
dht_max_age: 15.0

[temperature_sensor druckkammer]
sensor_type: DHT22
sensor_pin: curbrd:PD2
dht_auxiliary_mcu: True
dht_report_time: 5.0
dht_max_age: 15.0
```

Nicht in produktive Dateien übernehmen, bevor MCU-Identität, Beschaltung,
Firmware-/Host-Version und Rückweg bestätigt sind. Ein `^` vor `sensor_pin`
aktiviert ausdrücklich den internen Pull-up. Dieser ist standardmäßig aus
und kein ungeprüfter Ersatz für den externen Widerstand. Invertierung und
Pull-down sind nicht unterstützt.

Messabstand: DHT11 größer als 1 s, DHT22 größer als 2 s, maximal 120 s.
`dht_max_age` muss größer als der Messabstand sein. Die Firmware wartet nach
Konfiguration mindestens 2 s vor dem ersten Start und verhindert auch bei
Stop/Restart zu schnelle neue Starts. Eine regelmäßige Messreihe behält
ihren Zeitraster bei Protokollfehlern bei.

Der Temperaturcallback erhält ausschließlich gültige Ergebnisse in °C mit
dem MCU-Zeitstempel des Messbeginns, umgerechnet auf Print-Time.
`min_temp`/`max_temp` werden als zusätzliche Akzeptanzgrenzen verwendet.
Verletzungen werden als `OUT_OF_RANGE` gezählt, ohne einen Heizungs- oder
DHT-bedingten Shutdown auszulösen. DHT als Extruder-, Bett-, generischer
Heizungs- oder Temperatur-Lüftersensor wird bereits im Host abgelehnt.

Zusätzlich werden `dht_status waermetauscher` und `dht_status druckkammer`
automatisch registriert. Dafür keine eigenen Konfigurationssektionen anlegen.
Beispiel in einem Makro:

```jinja
{% set dht = printer['dht_status druckkammer'] %}
{% if dht.valid %}
  {action_respond_info('T=%.1f C RH=%.1f %% Alter=%.1f s Fehler=%s'
                       % (dht.temperature, dht.humidity,
                          dht.age_seconds, dht.last_error))}
{% else %}
  {action_respond_info('DHT-Messwert fehlt oder ist veraltet')}
{% endif %}
```

| Feld | Bedeutung |
| --- | --- |
| `temperature`, `humidity` | °C / % rF; vor erstem Erfolg und ab Ablauf von `dht_max_age` jeweils `null` |
| `valid` | Letzter erfolgreicher Wert ist jünger als `dht_max_age` |
| `age_seconds` | Alter seit Messbeginn; vor erstem Erfolg `null` |
| `last_error` | Letzter Ergebnisstatus, anfangs `NOT_READY`; kann bei Staleness weiterhin `OK` sein |
| `ok_count` | Erfolgreiche Messungen |
| `checksum_errors` | Prüfsummenfehler |
| `timeout_errors` | Fehlende Antwort und abgebrochene Übertragung |
| `protocol_errors` | Puls-, Bus- und unbekannte Protokollfehler |
| `range_errors` | Werte außerhalb des Sensorbereichs oder der Hostgrenzen |

Nach einem einzelnen Fehler bleibt der letzte Erfolg bis zum Alterslimit
mit unverändertem Alter und ausdrücklichem `last_error` verfügbar. Fehler
werden niemals als 0 °C oder als neuer alter Messwert an den Callback gegeben.
Doppelte oder rückwärts laufende Ergebniszeitstempel werden ignoriert.

Die normale `[temperature_sensor]`-Anzeige und M105 puffern weiterhin die
letzte Temperatur, anfangs 0. Dieser vorhandene Klipper-Cache enthält kein
Gültigkeitsflag. Für Diagnose und Auswertung deshalb das `dht_status`-Objekt
verwenden. Die DHT-Werte dürfen unabhängig von der Anzeige nicht für
Heizungsregelung oder Sicherheitsentscheidungen verwendet werden.
Es gibt keinen Patch am Verhalten anderer Temperatursensoren.

Einzelne DHT-Protokollfehler sind normale Statusmeldungen. Ein physischer
MCU-Ausfall bleibt dagegen Klippers normale MCU-Verbindungsstörung und
kann weiterhin den gesamten Drucker anhalten.

## Firmware und Drahtprotokoll

Der plattformunabhängige Decoder liegt in `src/dht_protocol.c`; Scheduler,
OID und Meldungen in `src/dht.c`, AVR-GPIO/Interrupts in `src/avr/dht_avr.c`.
Die zusätzliche Datei trennt den deterministisch testbaren Protokollkern
vom Scheduler. Es werden keine Arduino-Bibliotheken verwendet.

Die MCU zieht DATA für DHT11 20 ms beziehungsweise für DHT22 2 ms LOW.
Danach wird der Pin zum Eingang. Ein Ausgang, der HIGH treibt, wird nie
eingestellt. Die mögliche eigene steigende Freigabeflanke ist ein eigener
Empfangszustand. Keine Busy-Wait-Synchronisation ist nötig.

ISR: Timer1-Zähler und Pinpegel lesen, Pinänderung prüfen, Decoder fortsetzen,
bei Abschluss IRQ abschalten und Task wecken. Kein CRC, `sendf`, Logging,
Float oder Laufzeit-Allokation in der ISR. Nach dem 40. HIGH-Puls ist der
Frame fertig; die normale letzte Freigabeflanke wird nicht mehr benötigt.
Prüfsumme und Umrechnung laufen im Task. Klippers vorhandener Startup-Arena
wird einmalig ein OID-Objekt entnommen; pro Messung wird nichts allokiert.

Timer1-Konfiguration und Klippers zentraler Scheduler bleiben unverändert.
16-Bit-Differenzen erfassen nur kurze Flankenabstände. Ein Scheduler-Guard
alle 250 µs verwirft Empfangslücken über 200 µs; dadurch können lange Lücken
nicht nach 4096 µs als kurze gültige Pulse erscheinen. Zusätzlich gilt eine
32-Bit-Gesamtfrist von 8 ms nach Freigabe. Das setzt wie Klippers übrige
Timerfunktionen voraus, dass Interrupts rechtzeitig abgearbeitet werden.
Millisekundenlange fremde IRQ-Sperren sind nicht unterstützt.

| Phase | Akzeptierter Abstand |
| --- | --- |
| Eigene steigende Freigabeflanke | höchstens 50 µs seit Freigabe |
| Beginn der Sensorantwort | 10–160 µs seit Freigabe |
| Antwort LOW und HIGH | jeweils 45–120 µs |
| Bit LOW | 25–85 µs |
| Bit HIGH | 10–100 µs; ab 50 µs Bit 1 |

Die Grenzen erlauben Jitter, garantieren aber keine Erkennung jedes beliebig
verfälschten Signals: Die Sensorprüfsumme ist nur acht Bit breit. Eine
Spannungs- oder Anstiegszeitmessung kann der digitale Decoder nicht ersetzen.

DHT22: RH als Binärwort in 0,1 %, Temperatur als Betrag mit Vorzeichenbit in
0,1 °C; akzeptiert werden −40 bis 80 °C und 0 bis 100 % rF.
DHT11: `10*b0+b1` und `10*b2+b3`, dezimale Bytes jeweils 0–9; akzeptiert
werden 0 bis 50 °C und 20 bis 90 % rF. Varianten mit anderem Dezimalformat
oder negativem Temperaturformat werden bewusst verworfen. Die nutzbare
Feuchtespezifikation des DHT11 ist zusätzlich temperaturabhängig.

```
config_dht oid=%c pin=%u sensor_type=%c pull_up=%c
query_dht oid=%c clock=%u rest_ticks=%u
dht_result oid=%c clock=%u status=%c temp10=%hi humidity10=%hu
```

`sensor_type` ist 11 oder 22. `query_dht` mit `rest_ticks=0` beendet die
Messung, entfernt den Timer, verwirft ausstehende Ergebnisse und gibt DATA
frei. Stop, Restart und Shutdown räumen ebenfalls IRQ/GPIO auf. Das
Shutdown-Hook folgt Klippers normalem Entfernen aller Benutzertimer.
`clock` ist der tatsächliche Messbeginn als 32-Bit-MCU-Zähler.
Bei einem Fehler sind die numerischen Nutzdaten bedeutungslos.

| Code | Status | Diagnose |
| --- | --- | --- |
| 0 | `OK` | 40 Bits, Pulsfolge, CRC und Wertebereich gültig |
| 1 | `NO_RESPONSE` | Keine vollständige Antwort begonnen |
| 2 | `TIMEOUT` | Empfang abgebrochen oder Gesamtfrist überschritten |
| 3 | `PULSE_ERROR` | Unpassende Flankenfolge oder Pulsdauer |
| 4 | `CHECKSUM` | Prüfsumme falsch |
| 5 | `OUT_OF_RANGE` | Sensordatenformat oder Wertebereich unzulässig |
| 6 | `BUS_BUSY` | DATA schon vor dem Start LOW |

Unzulässige Pins, zweite Sensoren und belegte IRQs sind Konfigurationsfehler
mit verständlichem Fehlertext. Dafür gibt es keinen Laufzeitstatus
`UNSUPPORTED_PIN`. Die Host-Pinreservierung erfasst GPIO-, Alias- und
SPI-Konflikte; das Backend prüft zusätzlich INT0, PCINT0 und aktiviertes SPI.
Für PCINT0 existiert hier kein allgemeiner Vektorverteiler: bereits belegte
Gruppe wird abgelehnt. Die ISR vergleicht trotzdem nur PB5; andere Maskenbits
und Registerbits bleiben erhalten. Eine konkurrierende Vektordefinition
scheitert beim Linken statt einen bestehenden Handler zu überschreiben.

## Offline-Build und Tests

Im Repository mit einer vorhandenen AVR-Toolchain und Klipper-Testumgebung:

```sh
python3 -m unittest discover -s test -p 'test_*.py'
python3 scripts/test_dht_build.py
```

Der Buildtest erzeugt ausschließlich `out/dht-build/`: DHT aus/an,
unveränderte AVR-Regressionen für ATmega328 und ATmega2560 sowie Kopien der
beiden RF2000-Nano-Buildprofile aus/an. Gerätekennungen werden aus diesen
Testkopien entfernt. Die originalen Buildprofile werden nicht verändert.
Das neue `test/configs/atmega328p-dht.config` wird auch automatisch von
Klippers bestehender CI-Buildschleife erfasst.

Der Buildtest prüft Kommandowörterbuch und Vektorsymbole, speichert
`avr-size`, Symboltabellen, vollständiges Disassembly und eine konservative
CFG-Obergrenze für beide ISRs. Windows nutzt vorhandenes PlatformIO AVR-GCC,
Git-Shell und das lokale Test-Make; Include-Verzeichnisse werden dort kopiert,
weil Git unter Windows Verzeichnis-Symlinks emuliert. Die einzige zusätzliche
Buildgenerator-Korrektur entfernt ein CR aus Tool-Versionszeilen, damit
Windows-CRLF nicht die generierte C-Kommentarzeile beendet.

Für die echte Klippy-Integration nach dem CI-Build:

```sh
python3 klippy/klippy.py --import-test
python3 scripts/test_klippy.py -d ci_build/dict test/klippy/dht.test
```

Die Integrationskonfiguration verwendet zwei Nanos mit separaten OIDs,
beide Sensorfactories, alle sechs ADC-Kanäle des zweiten Nanos und dessen
PB1/PB3-Ausgänge sowie PB2/PB4-Statuseingänge. Sie kann nur die Konfiguration
und generierten Befehle prüfen, keine realen ADC-Werte oder Schaltvorgänge.

Konkrete Ergebnisse, Ressourcen und offene Punkte:
[DHT-Testbericht](DHT-Testbericht.md).

## Hardwareabnahme und Rückweg – nur nach gesonderter Freigabe

Die folgenden Schritte sind eine Anleitung. Sie wurden nicht ausgeführt.
Die Implementierungsfreigabe umfasst keine Produktivinstallation.

1. Laufenden Hostcommit und Firmwareversionen aus dem echten `klippy.log`
   sichern und mit dem lokalen Quellstand abgleichen. MCU-Namen, serielle
   Pfade, Chipbeschriftung, 16-MHz-Takt und Bootloader-Protokoll/-Baudrate
   vor Ort prüfen. Keine seriellen Pfade raten oder aus Beispielen kopieren.
2. Alle produktiven Konfigurationen und die originalen passenden
   Applikations-HEX-Dateien mit SHA256 sichern. Fehlen diese, zuerst einen
   verifizierten Wiederherstellungsweg herstellen. Lokale Quellkopien und
   unbekannte alte Buildausgaben sind kein Firmwarebackup.
3. Aufgabenliste für beide Nanos bestätigen: keine Stepper; ADC, Tachos,
   Display/I2C und Software-PWM auf `wtrbrd`; Strommessung und geschaltete
   Peltier-Leistung auf `curbrd`. D9/D11 sowie der gesamte Wasserkühlkreis
   müssen für Wartung in einem bestätigten sicheren Zustand sein.
4. PB5 als frei von SPI bestätigen, Pull-up und HIGH-Pegel mit der D13-LED
   messen. Ohne saubere Pegel keine Freigabe für den ersten Sensor.
5. Hostpatch und Firmware auf dem **zur tatsächlich installierten Version
   passenden** Quellstand bauen und offline prüfen. Bei Abweichung zum
   hier getesteten Commit diese Prüfungen wiederholen. Das Minimalprofil
   nicht ungeprüft anstelle des bestehenden Nano-Profils verwenden.
6. Nur mit expliziter Freigabe Dienste stoppen, freigegebenen Nano flashen,
   dessen Sensorsektion ergänzen und Dienste starten. Zuerst `wtrbrd` mit
   DHT11/PB5, danach gesondert `curbrd` mit DHT22/PD2. Die Haupt-MCU wird
   nicht geflasht. Das vorhandene Sammel-Updateskript nicht verwenden.

Beispiel für den dann separat freizugebenden Flashbefehl; alle Variablen
müssen zuvor mit den verifizierten Daten des **einen** Nanos gesetzt werden:

```sh
avrdude -p atmega328p -c arduino \
  -P "$DHT_VERIFIED_PORT" -b "$DHT_VERIFIED_BOOTLOADER_BAUD" \
  -D -U "flash:w:$DHT_REVIEWED_APPLICATION_HEX:i"
```

Baudrate/Protokoll aus dem tatsächlichen Bootloader bestimmen, nicht aus
Klippers serieller Laufzeit-Baudrate ableiten. Signaturprüfung beibehalten.
Auch ein Auslesebefehl über den Bootloader kann den Nano zurücksetzen und
gehört deshalb in die gesondert freizugebende Wartung.

Nach jedem einzelnen Nano zuerst mindestens 100 Zyklen messen, dann einen
Lauf von mindestens zwei Stunden, bei 5 s etwa 1440 Messungen pro Sensor.
Ziel bei stabiler Hardware: mindestens 99 % valide Rahmen. Zählerdifferenzen,
ADC-Werte, alle Ausgänge, CPU-Last und Klippers Log auf `Timer too close`
oder Shutdowns auswerten. Sensorunterbrechungen nur kontrolliert und gefahrlos
prüfen. Reale IRQ-Latenz und D13-Pegel sind ausdrücklich noch abzunehmen.

Bei einer Regression zuerst DHT-Abfrage beenden und den sicheren
Wartungszustand herstellen. Unter gesonderter Freigabe die gesicherten
Konfigurationen und ursprüngliche Hostversion wiederherstellen, den
betroffenen Nano mit seiner gesicherten originalen Applikations-HEX über
denselben verifizierten Einzelgerätebefehl zurücksetzen, danach Dienste
starten und ADC-/Ausgangsfunktion erneut prüfen. Flashdatei und Prüfsumme
müssen zu diesem Nano gehören. Keine Wiederherstellung anderer MCUs
auslösen. Ohne verifiziertes Originalartefakt keine Hardwareabnahme beginnen.

## Datenblätter

- [DHT11-Datenblatt, übersetzte Herstellerangaben](https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf):
  getrennte Ganzzahl-/Dezimalbytes, 0–50 °C, 20–90 % rF, Startsignal und Pull-up.
- [Aosong AM2302 Product Manual](https://blog.eletrogate.com/wp-content/uploads/2019/01/Datasheet_DHT22_AM2302.pdf):
  Open-Drain/Tristate, mindestens 2 s Messabstand und Temperatur-Vorzeichenbit.
  Die hier akzeptierten 100 % rF berücksichtigen übliche DHT22-Varianten;
  dieses AM2302-Dokument spezifiziert die Obergrenze als 99,9 % rF.
