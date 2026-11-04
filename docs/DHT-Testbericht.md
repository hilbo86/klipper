# DHT11/DHT22: schlanker AVR-Treiber

Stand: 04.11.2026. Der abgenommene schlanke Laufzeitcode entspricht
`feature/dht-avr` bei `af2a6811a` und wird nach `TH-Mod` übernommen.

## Ziel und Hardwarestand

Die Sensoren liefern langsame Umgebungsgrößen auf den beiden RF2000-Nanos.
Fehlgeschlagene Einzelmessungen werden akzeptiert. Der letzte erfolgreiche
Wert bleibt innerhalb des konfigurierten Alterslimits verfügbar; ein Fehler
erneuert dessen Zeitstempel nicht. Für den RF2000 sind 90 s beim DHT11 und
30 s beim DHT22 vorgesehen. Nach längerem Ausfall werden die Werte weiterhin
ungültig.

Der Nutzer hat den schlanken Build auf dem RF2000 getestet und das Ergebnis
als ausreichend für die Übernahme nach `TH-Mod` freigegeben:

| Sensor | Erfolgreich | Prüfsummenfehler | Protokollfehler | Timeout | Gesamt | Erfolgsquote |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DHT11 `temp_ambient` | 129 | 3 | 34 | 0 | 166 | 77,71 % |
| DHT22 `temp_chamber_below` | 166 | 0 | 0 | 0 | 166 | 100 % |

Beide Werte waren beim Abruf gültig, der letzte Erfolg jeweils weniger als
5 s alt. Die DHT11-Fehlerquote von 22,29 % wird für die träge Umgebungsgröße
mit dem vorgesehenen 90-s-Alterslimit ausdrücklich akzeptiert.

Der DHT11 ist laut Nutzer ein einzelner Sensor mit 3 cm PCB-Leiterbahn und
1-kΩ-Pull-up. Der `wtrbrd` betreibt zusätzlich ADCs, Tachos, Software-PWM und
ein I²C-Display. Die zuletzt getestete Timerdiagnose führte zu einer starken
Verschlechterung: Nach Aktivierung blieb `ok_count` bei 22, während das
Alter des letzten Erfolgs auf über 250 s stieg. Ihre null Intervalltreffer
bewiesen keine störungsfreie Flankenerfassung. Auf Wunsch des Nutzers wird
dieser Diagnoseansatz beendet und aus dem Laufzeitcode entfernt.

## Bereinigung

- Alle MCU-Puls-/Timerdiagnosekommandos und Trace-Datensätze entfernt.
  Keine Debug-Erfassung im Interrupt und keine Probe im AVR-Timerhandler.
- Der AVR-Timerhandler entspricht wieder der Quellbasis ohne DHT-Hooks.
- Der Flankenpfad verwendet das bereits aktive GPIO-Objekt direkt; der
  zusätzliche globale Sensorzeiger und die redundante Zustandsprüfung
  entfallen. Das GPIO-Backend ruft ihn nur für den aktivierten Empfänger auf.
- Experimentelles DHT11-Sonderfenster vor dem Prüfsummenbyte entfernt.
  Dadurch entfällt die sensorabhängige Verzweigung im Pulsdecoder.
- Empfangs-Watchdog von 250 auf 1000 µs reduziert. Fehlende Flanken werden
  weiterhin vor dem 4096-µs-TCNT1-Überlauf abgefangen. Drei Viertel der
  periodischen Kontrollen entfallen. Der normale Testframe benötigt drei
  Empfangskontrollen statt 14.
- Redundanten 32-Bit-Gesamtzeitvergleich entfernt. Die akzeptierten Phasen
  begrenzen einen vollständigen Rahmen bereits auf höchstens
  `160 + 120 + 120 + 40*(85 + 100) = 7800 µs`. Voraussetzung bleiben
  rechtzeitig bearbeitete Interrupts wie bei Klippers übrigen Timerfunktionen.
- Nachgewiesene Korrektur des Empfangsstarts bleibt erhalten: Puffer vor
  Busfreigabe initialisieren und Pegel nach IRQ-Aktivierung erneut prüfen.
- Host-Puls-/Timingdiagnose und Trace-Umschaltung entfernt. Der normale
  `QUERY_DHT SENSOR=<Name>` liest weiterhin ausschließlich den Hoststatus.

Erhalten bleiben Open-Drain-Ausgabe, 40-Bit-Rahmenprüfung, Pulsfolge,
Prüfsumme und Wertebereichsprüfung, Warm-up und Messabstände, gültige/stale
Werte, Fehlerzähler sowie Pin-/MCU-Konfliktprüfungen. Prüfsumme und Umrechnung
laufen im Task; keine Fließkommaoperation, serielle Ausgabe oder Allokation
im Empfangsinterrupt. `dht_result`, Statuscodes und `DHT_VERSION=1` bleiben
kompatibel. Die Firmware benötigt nur `config_dht` und `query_dht`.

## Offline-Prüfungen

- **134 Unittests bestanden**, darunter 22 C-Szenarien mit Produktionsdecoder,
  Schedulerclient und AVR-GPIO/IRQ-Backend.
- Abgedeckt sind DHT11/DHT22-Formate, Vorzeichen, Prüfsumme und Wertebereich,
  Pulsgrenzen, fehlende Flanken, TCNT1-/32-Bit-Überläufe, Stop/Restart/Shutdown,
  Pin-Konflikte, ausbleibende Antwort und beide Empfangsstart-Rennen.
- Ein nahezu maximaler synthetischer Rahmen endet nach 7698 µs und wird
  anschließend korrekt als außerhalb des Wertebereichs abgelehnt. Ein
  zusätzlicher Gesamtzeit-Timer ist dafür nicht erforderlich.
- Ein Hosttest prüft 30-/90-s-Altersgrenzen ausdrücklich über 18 aufeinander
  folgende Fehler: letzter Erfolg und Zeitstempel bleiben unverändert,
  Gültigkeit endet genau am jeweiligen Limit.
- **Acht AVR-Builds bestanden**: DHT aus/an, bestehende ATmega328-/2560-
  Regressionen und beide bisherigen Nano-Profile jeweils aus/an.
- Wörterbücher und Symbole enthalten keine DHT-Debug-/Tracekommandos und
  keine Timerprobe. Der Buildtest prüft deren Abwesenheit ausdrücklich.
- **Sieben Klippy-Integrationstests bestanden** mit simulierten Nanos und
  echten MCU-Wörterbüchern, einschließlich ADC-/Ausgangskoexistenz und
  normaler Konsolenabfrage.

## Ressourcen

Beide vollständigen Nano-Profile ergeben dieselben Ressourcen:

| Stand | `.text` | `.data` | `.bss` | DHT-OID-Objekt |
| --- | ---: | ---: | ---: | ---: |
| Vorheriger Timerdiagnosebuild | 19482 | 42 | 387 | 92 |
| Schlanker Build | 18196 | 42 | 363 | 40 |

Ersparnis gegenüber dem Diagnosebuild: **1286 Byte Flash und 76 Byte
dauerhafter RAM**. Der gesamte DHT-RAM-Mehrbedarf gegenüber dem Profil ohne
DHT beträgt jetzt 47 Byte: 40 Byte OID-Objekt, 4 Byte OID-Eintrag und 3 Byte
statisch. Die Objektgröße wird am tatsächlichen AVR-Allokationsaufruf geprüft.

Konservative Obergrenze beider DHT-ISRs aus dem Disassembly: 254 Takte,
15,875 µs. Fremde Interruptlatenz ist darin weiterhin nicht enthalten.
Die bisherigen Nano-Buildprofile und leeren Startpin-Tabellen bleiben erhalten.

## Mainsail und Lieferung

Die DHT-Feuchteanzeige wurde mit DCO-Signoff als `ccba21e9` committed und
direkt nach `TH-Mod` übernommen und gepusht, wie vom Nutzer klargestellt.
Die Änderung umfasst drei Produktionszeilen und fünf Tests für Zuordnung,
reine Feuchteanzeige, fehlende/veraltete Werte und unveränderten AHT-Fallback.
Alle 60 Mainsail-Tests, Formatierung, Lint und lokaler Produktionsbuild
bestanden. Unabhängige Formatänderungen des vollständigen Formatlaufs
wurden vor dem Commit wieder entfernt.

Der bestehende Releaseworkflow ist erfolgreich abgeschlossen und hat
[v2026.1104.69](https://github.com/hilbo86/mainsail/releases/tag/v2026.1104.69)
mit installierbarem `mainsail.zip` veröffentlicht. Zielcommit ist `ccba21e9`.

Das Nano-Paket liegt unter `out/dht-slim/DHT-slim-nanos.zip`. Es enthält
beide neuen HEX-/Wörterbuch-/Buildprofile, den passenden schlanken Hostadapter,
Konfigurationsmuster, Anleitung, Quellpatch und Prüfsummen. Die vorherigen
Diagnosefirmwares und der vorherige Hostadapter sind als Rückweg enthalten.
Sie sind kein Backup der Installation vor der DHT-Erweiterung.

Alle vorherigen lokalen Änderungen und Buildartefakte wurden vor der
Bereinigung mit SHA256 unter `../dht-backups/2026-11-04/pre-slim` gesichert.
Produktive RF2000-Boardkonfigurationen wurden nicht verändert; die
Altersgrenzen setzt der Nutzer dort selbst. Nano-Flashen und Dienstneustarts
wurden nicht automatisch ausgeführt. Die Übernahme nach `TH-Mod` und dessen
Veröffentlichung erfolgen nach ausdrücklicher Freigabe des Nutzers. Der
Laufzeitcode entspricht dem bereits geflashten und getesteten Featurestand;
der Branchwechsel am Host erfordert deshalb kein erneutes Nano-Flashen.
