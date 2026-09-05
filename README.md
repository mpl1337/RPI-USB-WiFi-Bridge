# RPI USB WiFi Bridge v1.0

**RPI USB WiFi Bridge** turns an ESP32-S3 with USB Host into a WiFi bridge for Raspberry Pi RP2040/RP2350/RP2354 based devices, including OpenKNX modules.

It provides a browser terminal, an optional separate console window, Raw TCP/RFC2217 access, USB diagnostics, RP-series BOOTSEL/UF2 flashing and ESP32 OTA updates over WLAN.

## Features

- USB CDC to browser/WebSocket bridge
- Browser console with VT100/ANSI support and long scrollback
- Optional console in a separate browser window
- Raw TCP and RFC2217 on port `3333`
- RP2040 and RP2350/RP2354 BOOTSEL detection
- UF2 firmware upload from the browser
- UF2 firmware analysis before flashing
- ESP32-S3 firmware update via Web-OTA
- Automatic USB reconnect with diagnostic status
- Up to two browser terminals at the same time
- WiFi setup through an ESP32 access point with captive portal, stored in NVS

## Screenshots

### Web interface

The main page shows WiFi and USB status, the detected RP device, bridge statistics, BOOTSEL controls, firmware update functions and the integrated serial console.

![RPI USB WiFi Bridge web interface](screenshots/overview.png)

### RP firmware analysis

Before flashing, the uploaded UF2 file is analyzed. The bridge displays target information, UF2 structure details, flash range and detected OpenKNX modules where available.

![RP firmware analysis](screenshots/firmware-analysis.png)

### Browser console

The serial console can be used directly in the main interface or opened in a separate browser window.

![Browser console](screenshots/console.png)

## ESP-IDF installation

Use **ESP-IDF 6.1.x**. The project is developed with ESP-IDF 6.1 and the component manifest accepts versions `>=6.1,<6.2`.

### Windows

First install the **Espressif Installation Manager (EIM)**:

```powershell
winget install Espressif.EIM
```

This command installs **only EIM**, not ESP-IDF itself. If winget reports that the package is already installed, simply open **Espressif Installation Manager** from the Windows Start menu.

In EIM:

1. Add a new ESP-IDF installation.
2. Select an **ESP-IDF 6.1.x** release.
3. Complete the installation.
4. Open the ESP-IDF PowerShell/terminal created by EIM.

Verify the active ESP-IDF version:

```powershell
idf.py --version
```

The output should report ESP-IDF **6.1.x**.

Official installation guide: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/

## Build and flash

The supplied configuration is prepared for an ESP32-S3 with USB Host support and **16 MB flash**. For a fresh checkout:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py flash
```

Run `idf.py set-target esp32s3` only for a fresh/unconfigured checkout. It regenerates `sdkconfig`.

To completely erase the ESP32-S3 flash, including stored NVS data and WiFi credentials:

```powershell
idf.py erase-flash
idf.py flash
```

## WiFi setup

WiFi credentials are not compiled into the firmware and do not need to be entered in `menuconfig`.

After the first start without stored credentials, the bridge creates an open setup access point:

```text
RPI-USB-WiFi-XXXXXX
```

The last six characters are derived from the ESP32-S3 MAC address. The setup access point implements a captive portal. Android, iOS and Windows should normally offer or open the setup page automatically after connecting. If the portal is not shown, open:

```text
http://192.168.4.1
```

The captive portal uses DHCP option 114, redirects DNS requests to the ESP32-S3 and redirects unknown HTTP paths to the setup page. The setup page scans for nearby WiFi networks. Select the desired SSID, enter the password and press **Speichern und verbinden**. The credentials are stored in NVS and the bridge restarts automatically.

If stored WiFi credentials no longer work, the bridge retries the connection several times and then starts the setup access point again automatically.

The setup access point is only active while WiFi configuration is required. From the normal web interface, **ESP32 Bridge-Firmware aktualisieren → WLAN neu einrichten** can be used to start the setup access point manually when changing networks.

## Additional documentation

- [Flash Nuke](FLASH_NUKE.md)

## License

The project source code is released under the **MIT License**. See [LICENSE](LICENSE).

Third-party components and downloaded build assets keep their respective licenses. In particular, `nuke_universal.uf2` is fetched from Raspberry Pi's `pico-sdk-prebuilts` during the build and is not relicensed by this project. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
