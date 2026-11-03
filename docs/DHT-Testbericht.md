# DHT-Implementierung: Änderungs- und Testbericht

Datum: 03.11.2026. Ergebnis: Softwareimplementierung und Offline-Prüfungen
abgeschlossen; Hardwareabnahme noch offen. Bedienung und manuelle
Flash-/Rollback-Schritte stehen in [DHT.md](DHT.md).

## Ausgangssituation und Sicherung

- Repository: lokaler Klipper-Checkout, Ausgangsbranch `TH-Mod`.
- Basiscommit: `d53ce4fccf19fcc1dec9de539b855c3d5728643e`.
- Arbeitsbranch: `feature/dht-avr`; Änderungen liegen als Arbeitskopie vor.
  Der Klipper-Checkout war vor Beginn sauber.
- Tatsächlich installierte Host-/MCU-Versionen, Chipbeschriftungen,
  Oszillatoren und Bootloader sind offline nicht bestätigt.
- Beide vorhandenen Nano-Buildprofile wählen ATmega328P, 16 MHz, keine
  Stepper, kein Hardware-PWM und kein SPI. ADC, I2C, Buttons und
  Pulszähler sind vorhanden; Neopixel ist mitkompiliert, im untersuchten
  Nano-Konfigurationsausschnitt jedoch nicht benutzt.
- Lokale RF2000-Konfiguration: `wtrbrd` für Wärmetauscher/Kühlkreis und
  `curbrd` für Strommessung/Leistungsschalter. Die Zuordnung zu den
  angeschlossenen DHTs muss vor Ort bestätigt werden.

Die 30 lokalen RF2000-Konfigurations- und Nano-Builddateien wurden zunächst
unter `out/dht-prechange` und anschließend dauerhaft außerhalb des
Buildverzeichnisses unter `../dht-backups/2026-11-03` gesichert.
`sha256.csv` enthält die privaten Sicherungsprüfsummen; alle 30 Dateien
wurden ohne Abweichung nachgeprüft. Diese Sicherung ist nicht eingecheckt
und enthält keine bestätigten Abbilder der installierten MCU-Firmware.
Gerätekennungen werden im Bericht und in neuen Beispiel-/Testdateien
nicht veröffentlicht.

Die originale Firmware und die laufende Konfiguration müssen vor einem
Produktivtest zusätzlich **am Gerät** gesichert und verifiziert werden.
Ein verwendbarer Rückweg auf die tatsächlich installierte Firmware ist
deshalb noch kein abgenommenes Ergebnis dieser Offline-Arbeit.

## Änderungen

| Dateien | Inhalt |
| --- | --- |
| `src/dht_protocol.c`, `src/dht.h` | Testbarer 40-Bit-Decoder, Plausibilitätsgrenzen, CRC und typspezifische Ganzzahlkonvertierung |
| `src/dht.c` | OID-Kommandos, Startpuls, Scheduler-Guard, Empfangsfrist, Task-Reports, Stop/Restart/Shutdown |
| `src/avr/dht_avr.c` | PB5/PCINT5 und PD2/INT0, Eingang/LOW-Steuerung, Masken- und Pinprüfung |
| `src/Kconfig`, beide Makefiles | Optionale Einbindung nur für den unterstützten MCU-/Takt-/Funktionsumfang |
| `klippy/extras/dht_sensor.py`, `temperature_sensors.cfg` | Sensorfactories, Pinbelegung, Zeitstempel, Fehlerzähler und getrennte Statusobjekte |
| `config/sample-dht.cfg` | Kommentiertes Beispiel ohne reale Gerätekennungen |
| `test/configs/atmega328p-dht.config` | Zusätzliches AVR-CI-Buildziel |
| `test/test_dht*.py` | Produktiver C-Code im Register-/Zeitmodell, Hostadapter und reale Kconfig |
| `test/klippy/dht.cfg`, `dht.test` | Echte Klippy-Integration mit zwei MCUs, sechs ADC-Eingängen und Schaltpins |
| `scripts/test_dht_build.py` | Acht isolierte Builds, Wörterbücher, Symbole, Ressourcen und ISR-CFG-Analyse |
| `scripts/buildcommands.py` | CR am Ende nativer Tool-Versionszeilen entfernen, damit generiertes C unter Windows gültig bleibt |
| `docs/DHT*.md` | Bedienung, Grenzen, Testnachweis und manuelle Abnahme/Rollback |

Die Abweichung vom vorgeschlagenen Dateiplan ist die separate
`dht_protocol.c`: derselbe Decoder läuft so unabhängig von Scheduler und
AVR sowohl im Modelltest als auch in der Firmware. Es gibt keinen Patch
an Timer1, am zentralen Scheduler oder an anderen Temperatursensoren.

## Architektur- und Konfliktprüfung

Im Basisquellstand waren `INT0_vect`, `PCINT0_vect`, `EIMSK`, `EICRA`,
`PCMSK0` und `PCICR` für diese Aufgabe frei. Timer1 ist Klippers vorhandene
freilaufende Zeitbasis ohne Vorteiler; der neue Treiber liest nur `TCNT1`.
`TCCR1A/B`, `TIMSK1`, `OCR1A/B` und `TCNT1` werden vom DHT-Code nicht
beschrieben. Eine zweite Vektordefinition wäre ein Linkfehler.

Host-Pinreservierung und der vorhandene SPI-Pinresolver verhindern
Mehrfachbelegungen einschließlich Alias-/SPI-Konflikten. Das Backend lehnt
unzulässige Pins, mehr als einen Sensor, eine belegte PCINT0-Gruppe, ein
belegtes INT0 und aktiviertes SPI auf PB5 ab. Bei einer abgewiesenen
Pinkonfiguration wird kein uninitialisiertes GPIO-Objekt im Shutdown benutzt.

Die lokale Konfiguration belegt `curbrd:PB1/PB3` als `power_output` für
Peltier-Leistung, `PC1/PC0` für deren Strommessung, `PC4/PC5` für weitere
Strommessungen, `PE2/PE3` für A6/A7-Spannungsmessung und `PB2/PB4` als
Statuseingänge. Diese Pinbelegungen bleiben erhalten. D10/D12 sind in
diesem Klipper-Stand Eingänge, obwohl die historische Headerbeschreibung
sie als Statusausgänge bezeichnet. Die tatsächliche Signalrichtung ist
vor der Hardwareabnahme zu klären.

`wtrbrd` betreibt zusätzliche ADCs, Pumpen/Lüfter, Tachos, Peltier-Ausgänge
und I2C-Bausteine einschließlich Display. PB5 war im untersuchten lokalen
Konfigurationsstand frei, PD2 ist dort bereits ein Peltier-Ausgang und darf
nicht für den zweiten Sensor verwendet werden. PD2 für DHT22 ist für
`curbrd` vorgesehen. Die DHT-Pinbelegung wird nicht automatisch produktiv
ergänzt oder aus diesen Befunden als physisch bestätigt behandelt.

## Tatsächlich ausgeführte Prüfungen

```sh
python3 -m unittest discover -s test -p 'test_*.py'
python3 scripts/test_dht_build.py
python3 klippy/klippy.py --import-test
python3 scripts/test_klippy.py -d out/dht-build/dict \
  test/klippy/dht.test test/klippy/temperature.test \
  test/klippy/power_output.test \
  test/klippy/analog_input_invalid_sampling.test \
  test/klippy/commands.test test/klippy/pwm.test test/klippy/macros.test
```

| Prüfung | Ergebnis |
| --- | --- |
| Python-/C-Modell-Unittests | **131 bestanden**, einschließlich 13 DHT-Testmethoden |
| C-Modell des produktiven Decoders, Treibers und AVR-Backends | **18 Szenarien bestanden**, mit zusätzlichen Schleifen für Formate, Jitter, Zustände und Fehler |
| Kconfig | Unterstützte Konfiguration aktiv; Default, andere Chips/Takte, Stepper und Hardware-PWM abgelehnt |
| Vollständige Firmware-Builds | **8 bestanden**, siehe Ressourcen unten |
| Host-Importtest unter Linux | **Bestanden** |
| Klippy-Integration und bestehende relevante Regressionen | **Alle 7 bestanden** |
| Whitespace-Prüfung neuer Dateien / `git diff --check` | **Bestanden** |
| Prüfsummen der dauerhaften lokalen Sicherung | **30 geprüft, 0 Abweichungen** |
| Flash, produktive Konfiguration, Dienstneustart | **Nicht ausgeführt** |
| Reale DHT-/ADC-/Ausgangsfunktion und Dauerlast | **Nicht geprüft** |

Die Unittests liefen mit Windows-Python 3.11.7 und dem vorhandenen
GCC-13.1.0-Hostcompiler. Der Registertest kompiliert und führt den
Produktionscode aus; nur Plattformheader, Hardware und Schedulerdienste
werden modelliert. Keine ISR wird im Test durch eine zweite Treiberkopie
ersetzt. Geprüft werden unter anderem:

- DHT11-Dezimalbytes, positive und negative DHT22-Temperatur, RH-Nachkommastellen.
- 40 Bits MSB-first, Null-/Eins-/Wechselmuster, 49-/50-µs-Schwelle und Jitter.
- CRC, Wertebereiche, fehlende und verlorene Flanken, falsche Pulse,
  ausbleibende Antwort, Teilrahmen, dauerhaft HIGH/LOW und belegter Bus.
- Eigene Freigabeflanke, Abschluss am 40. Bit, ignorierte späte Flanken.
- 16-Bit-Überlauf, 32-Bit-Überlauf und lange Lücke, die sonst nach Timerumlauf
  als gültiger Puls erscheinen könnte.
- Stop/Restart/Shutdown in allen Empfangszuständen und bei ausstehendem
  Ergebnis; keine doppelten Reports oder doppelt eingehängten Timer.
- Neustart nach langem Stillstand und Mindestabstand nach abgebrochenem Start.
- Fremde Register-/Maskenbits, Filterung von Gruppeninterrupts und Pull-up.
- Zwei unabhängige Host-MCUs/OIDs, echte Nachrichtenkodierung einschließlich
  signed `%hi`, Print-Time, Fehlerzählung, Alter, Duplikate und MCU-Zeitepoche.

Die vollständigen Klippy-Tests liefen unter WSL/Linux mit Python 3.8.10.
Die fehlenden Pakete greenlet 2.0.2 und cffi 1.14.6 samt pycparser wurden als
Wheels ausschließlich in Projekt-Testverzeichnisse geladen. Kein Systempaket
und keine Druckerinstallation wurde geändert. Ein erster Lauf auf dem großen
NTFS-Checkout war beim Git-Metadatenscan sehr langsam; der erfolgreiche
Abschlusslauf verwendete deshalb eine temporäre Kopie derselben Quellen,
Konfigurationen und der erzeugten Wörterbücher. Ein vorläufiger Lauf hatte
noch keine eingebundenen Beispielkonfigurationen kopiert; nach deren
Ergänzung bestanden alle sieben Tests. Der native Windows-Importversuch
war erwartungsgemäß wegen des fehlenden Unix-Moduls `termios` ungeeignet.

Die gesamte plattformübergreifende Klipper-CI wurde nicht lokal ausgeführt:
dafür sind weitere MCU-Toolchains und Wörterbücher erforderlich. Die neue
Buildkonfiguration und der neue Klippy-Test sind in die vorhandenen
CI-Dateischleifen integriert und werden beim regulären CI-Lauf mitgeprüft.

## Gemessene Ressourcen und ISR

AVR GCC 7.3.0, GNU Binutils 2.26.20160125, bestehende Makefile-Optimierung
einschließlich LTO. `avr-size` des abgeschlossenen Messlaufs:

| Build | `.text` | `.data` | `.bss` |
| --- | ---: | ---: | ---: |
| DHT-Testprofil aus | 13700 | 42 | 359 |
| DHT-Testprofil an | 16068 | 42 | 364 |
| Vorhandener ATmega328-Regressionstest | 12348 | 42 | 356 |
| Vorhandener ATmega2560-Regressionstest | 45218 | 48 | 373 |
| Current-switch-Profil aus | 15870 | 42 | 360 |
| Current-switch-Profil mit DHT | 18218 | 42 | 365 |
| Peltier-controller-Profil aus | 15870 | 42 | 360 |
| Peltier-controller-Profil mit DHT | 18218 | 42 | 365 |

Flashmehrbedarf: 2368 Byte im Testprofil beziehungsweise 2348 Byte in den
Nano-Profilen, jeweils inklusive zusätzlichem Kommandowörterbuch. Zeit-/Versions-
und Quellstand im komprimierten Wörterbuch können die genaue Buildgröße ändern.
Es bleiben dieselben vorhandenen ADC-/GPIO-/I2C-Funktionen einkompiliert.

Zusätzlich zu den 5 Byte statischem RAM benötigt `struct dht` **44 Byte** aus
Klippers einmaliger Startarena; der zusätzliche OID-Tabelleneintrag benötigt
weitere 4 Byte. Damit sind es **53 Byte zusätzlicher dauerhafter RAM** pro
konfiguriertem Sensor/MCU, ohne Stack. Die Strukturgröße wurde am Größenargument
des erzeugten `oid_alloc`-Aufrufs im AVR-Disassembly geprüft. Es gibt weder
`malloc` noch eine Allokation pro Messung.

Beide Vektoren `__vector_1` / INT0 und `__vector_3` / PCINT0 sind im DHT-Build
stark definiert; in den Builds ohne DHT fehlen diese eigenen Definitionen.
Die automatisierte längste CFG-Pfadanalyse des AVR-Disassembly liefert für
beide Handler **248 Takte = 15,5 µs** als konservative Obergrenze inklusive
Hardwareeintritt, Vektorsprung, aufgerufener Funktionen und `RETI`.
Das sind 10,5 µs Abstand zu einem 26-µs-Puls, ohne fremde Interruptlatenz.
Die Analyse berücksichtigt auch längere Abschluss-/Fehlerpfade, verwirft
unbekannte Instruktionen oder Schleifen und behauptet keine gemessene
typische ISR-Laufzeit von nur wenigen Mikrosekunden.

Artefakte: `out/dht-build/sizes.json`, `*-symbols.txt`, `*-disassembly.txt`,
`*-irq-cycles.json`, die acht Buildlogs und `linux-integration.log`.
Die erzeugten HEX-Dateien sind Testartefakte und keine Freigabe zum Flashen.

## Offene Hardwareabnahme

- Reale Board-/Takt-/Bootloader- und Firmware-/Hostidentität bestätigen.
- Originalfirmware als passenden, prüfsummengesicherten Rückweg bereitstellen.
- D13-LED, Pull-up, Spannungspegel und Leitungskapazität messen.
- Signalrichtung D10/D12 und Bedeutung/Zustand von D9/D11 physisch prüfen.
- ISR-/Timerlatenz unter echten ADC-, I2C-, Tacho- und Schaltaktivitäten messen.
- Je Nano mindestens 100 Zyklen und anschließend mindestens 2 h testen;
  Fehlerquote, CPU-Last, ADCs und alle Ausgänge mit dem Ausgangsstand vergleichen.
- Kontrollierte Sensorunterbrechung, Staleness und Rückweg abnehmen.

Damit sind M1–M4 für die Software umgesetzt und offline geprüft.
Die produktive Bestandsaufnahme von M0 und M5 bleiben ausdrücklich offen.
Die Anleitung zur Freigabe und zum Einzelgeräte-Rollback steht in
[DHT.md](DHT.md#hardwareabnahme-und-rückweg--nur-nach-gesonderter-freigabe).

## Nachtrag: Konsolenabfrage und Luftfeuchtigkeit im Dashboard

Der Nutzer hat nach Installation der Nano-Firmwares und des Hostbranches
Temperaturen im Dashboard bestätigt. Elektrische Messungen und die übrige
Hardwareabnahme werden damit nicht als abgeschlossen behandelt.

Die Feuchtigkeit war bereits im Objekt `dht_status <Name>` vorhanden.
Mainsails Temperaturzeile erkennt zusätzliche Messwerte jedoch anhand einer
Liste bekannter Objektpräfixe; `dht_status` fehlte dort. Die Ergänzung im
Mainsail-Branch `feature/dht-avr` basiert auf `TH-Mod` bei `cf2d6a1b` und
umfasst drei Produktionszeilen: neues Präfix sowie Beschränkung auf
`humidity` in Anzeige und Sensoreinstellungen. Dadurch werden Flags und
Zähler nicht versehentlich als numerische Messwerte formatiert.
Feuchtigkeit ohne gültigen Wert bleibt ausgeblendet, die AHT-Anbindung bleibt
erhalten. Der generische Klipper-Temperatursensor wird weiterhin nicht geändert.

Im Klipper-Host wird zusätzlich `QUERY_DHT SENSOR=<Name>` automatisch pro
Sensor registriert. Der Befehl liefert Werte, Gültigkeit, Alter und alle
Fehlerzähler als JSON, ohne eine zusätzliche Messung auszulösen. Hostbasis
für diesen Nachtrag ist DHT-Commit `80058afb9`. Keine MCU-Quelldatei und kein
MCU-Nachrichtenformat werden durch diese Ergänzung geändert; die geflashten
Nano-Firmwares können weiterverwendet werden.

Nachtrag-Prüfungen:

- **133 Klipper-Unittests bestanden**, darunter Befehlsausführung im echten
  G-Code-Dispatcher, Sensorauswahl, unbekannter Sensor, Anfangszustand und
  Staleness bei wiederholter Diagnoseabfrage.
- **7 Klippy-Integrationstests bestanden**, nun einschließlich beider
  `QUERY_DHT`-Aufrufe mit simulierten Nanos.
- **60 Mainsail-Unittests bestanden**, darunter fünf neue Fälle für
  Sensorzuordnung, reine Feuchteanzeige und Einstellungen, Aktualisierung
  auf `null`, fehlende Anfangsmessung und unveränderten AHT-Fallback.
- ESLint, Prettier und vollständiger Mainsail-Produktionsbuild bestanden.
- Fertiges Frontendarchiv: `../../mainsail/dist/mainsail-dht.zip`,
  Buildkennung `v2.19.0-dht`; ZIP-Inhalt und Integrität geprüft.

Die Klipper- und Mainsail-Änderungen dieses Nachtrags liegen lokal auf den
jeweiligen `feature/dht-avr`-Branches. Eine Veröffentlichung auf GitHub oder
Installation der Oberfläche auf dem Drucker wurde nicht ausgeführt.

## Nachtrag: DHT11-Pulsfehler im Hardwaretest

Der Nutzer meldete folgende Zähler aus demselben Statusabruf:

| Sensor | Erfolgreich | CRC-Fehler | Protokoll-/Busfehler | Timeout | Gültige Quote |
| --- | ---: | ---: | ---: | ---: | ---: |
| DHT11 `temp_ambient` | 296 | 4 | 219 | 0 | 57,03 % von 519 |
| DHT22 `temp_chamber_below` | 519 | 0 | 0 | 0 | 100 % von 519 |

Beim DHT11 war das letzte Ergebnis `PULSE_ERROR`, der letzte erfolgreiche
Wert 13,98 s alt. `valid=true` folgt dem vereinbarten Alterslimit von 15 s
und behauptet nicht, dass der jüngste Rahmen erfolgreich war. Der Nutzer
bestätigte einen einzelnen DHT11, etwa 3 cm PCB-Leiterbahn und einen externen
1-kΩ-Pull-up. Eine schwach hochgezogene lange Leitung wird deshalb nicht
ohne weitere Messung als Ursache angenommen.

Die Quellprüfung ergab ein unabhängig davon vorhandenes Empfangsrennen:
Der alte Ablauf gab DATA frei, las den Pegel und initialisierte danach
den Decoder, bevor der IRQ aktiviert wurde. Eine steigende Flanke während
dieser Initialisierung konnte verloren gehen. Der erste Sensor-LOW wurde
dann bei gleichem gespeicherten Pegel ignoriert und ein späterer Puls als
Fehler interpretiert. Ein gezielt injizierter Flankenwechsel reproduziert
diesen Fehler im alten Code. Die korrigierte Initialisierung vor Freigabe
und die Pegelprüfung nach IRQ-Aktivierung bestehen diesen Test auf beiden
IRQ-Pfaden. Ein zweiter Test deckt die Lücke beim Aktivieren des IRQ ab.
Zusätzlich wurden die Scheduler-Berechnungen vor die Busfreigabe verlegt.

Damit ist ein Softwarefehler nachgewiesen und korrigiert. Dass er die
gemeldeten 219 Protokoll-/Busfehler erklärt, ist ohne erneuten Hardwarelauf
noch nicht nachgewiesen. Die Pulsgrenzen wurden bewusst nicht erweitert.

Eine neue optionale Abfrage `query_dht_diagnostics oid=%c` liest den letzten
Pulsfehler im Task-/Kommandokontext. Empfangszustand, Bitzahl, Tickabstand,
Pegel und Messbeginn werden bei `PULSE_ERROR` gesichert; kein `sendf` erfolgt
aus der ISR. Der Host stellt die Diagnose über
`QUERY_DHT SENSOR=temp_ambient DIAGNOSTICS=1` bereit. Das bisherige
`dht_result`-Format und `DHT_VERSION=1` bleiben kompatibel.

Neue Offline-Ergebnisse:

- 136 Unittests bestanden, darunter nun 21 C-Szenarien.
- Alle sieben Klippy-Integrationstests bestanden, einschließlich einer
  Detailabfrage mit `DIAGNOSTICS=1` gegen das neue MCU-Wörterbuch.
- Alle acht AVR-Builds bestanden; Nano-Profile mit DHT: `.text=18468`,
  `.data=42`, `.bss=365`. Der Flashmehrbedarf gegenüber dem Nano-Profil ohne
  DHT beträgt rund 2,6 KiB.
- Der AVR-Allokationsaufruf zeigt `sizeof(struct dht)=54`; mit 5 Byte
  statischem Mehrbedarf und 4 Byte OID-Eintrag sind es 63 Byte dauerhafter RAM.
- Konservative ISR-Obergrenze beider Vektoren: 255 Takte, etwa 15,94 µs.
- Die zuvor bereitgestellten Nano-ELF-/HEX-/Wörterbuchdateien wurden vor dem
  Neubau unter `out/dht-before-rx-fix` mit SHA256 gesichert und anschließend
  dauerhaft unter `../dht-backups/2026-11-03/pre-rx-fix` abgelegt.
- Testpaket: `out/dht-rx-fix/DHT11-rx-fix.zip`, mit neuer DHT11-Firmware,
  Hostadapter, vorheriger DHT-Firmware, Hostadapter aus `80058afb9`, Anleitung
  und SHA256-Prüfsummen. Diese vorherige DHT-Firmware ist kein Abbild der
  Installation vor Beginn der DHT-Erweiterung.

Für den ersten Wiederholungstest ist nur der DHT11-Nano vorgesehen. Der
fehlerfreie DHT22 muss dafür nicht aktualisiert werden. Neue Quote und
gegebenenfalls Pulsdiagnose müssen weiterhin am Gerät erhoben werden;
automatisches Flashen oder ein Dienstneustart wurden nicht ausgeführt.
