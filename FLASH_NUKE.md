# Flash Nuke

## Sicherheitsablauf

1. Warnhinweis bestätigen.
2. Exakt `NUKE` eingeben.
3. Der Browser sendet zusätzlich `X-Confirm-Nuke: NUKE`.
4. Ohne diesen Header lehnt die ESP32-API den Löschvorgang mit HTTP 428 ab.

## Technischer Ablauf

1. Der RP MCU wird bei Bedarf in RP BOOTSEL gebracht.
2. CDC-Reconnect bleibt während des Nuke-Zyklus gesperrt.
3. Die bereits in die ESP32-Firmware eingebettete `nuke_universal.uf2` wird direkt auf das RP-BOOTSEL-MSC-Geraet geschrieben.
4. Der universelle Nuke-Helper läuft auf RP2040 oder RP2350/RP2354 aus RAM und löscht den externen Flash.
5. Der Helper kehrt in BOOTSEL zurück; die Bridge wartet auf die erneute RP-BOOTSEL-Enumeration.

Zur Laufzeit ist **kein Internetzugang** nötig. Verwendet wird die gepinnte Raspberry-Pi-Datei aus `pico-sdk-prebuilts v2.3.0-0` (114688 Byte, SHA-256 `f1d80fcc37161df8cbdf16a58c111208f06234976a470c17699270d212e91754`). Falls die Binärdatei beim ersten Build noch nicht in `main/assets/` liegt, lädt CMake genau diese Version herunter, prüft den Hash und bettet sie anschließend in die ESP32-Anwendung ein.

## Firmware starten nach Flash Nuke

Nach einem erfolgreichen Flash Nuke ist der externe Flash des RP MCU leer. **Firmware starten** kann zwar den PICOBOOT-Reboot auslösen, aber ohne neu geflashte Anwendung gibt es nichts zu starten; der RP MCU kann deshalb wieder in BOOTSEL erscheinen.


Ab v0.7.2 wird die BOOTSEL-Chipfamilie über die ROM-USB-ID erkannt. Der eingebettete Raspberry-Pi-Helfer `nuke_universal.uf2` ist absichtlich generationenübergreifend und wird daher nicht durch die Single-Family-Prüfung für normale Benutzer-UF2-Dateien eingeschränkt.
