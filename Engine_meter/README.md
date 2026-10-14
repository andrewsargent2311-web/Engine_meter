# Engine_meter
nRF52840 (nice!nano) engine meter -- RPM + EGT/CHT over BLE (Nordic UART) to the paraglide_vario_ESP32.

## Build in VS Code (PlatformIO)
1. Install the **PlatformIO IDE** extension.
2. File > Open Folder... > select this `Engine_meter` folder.
3. Build: PlatformIO sidebar > `nicenano` > General > Build (or Ctrl+Alt+B).
4. Open the PlatformIO monitor for the nice!nano's COM port at 115200 baud.
   Start the monitor, then reset or reconnect the board to catch the startup
   message. Firmware waits up to 10 seconds for USB serial at boot; it continues
   running if no monitor is connected and prints sensor readings every 200 ms.

Firmware source is in `src/` (`main.cpp` and `AuxSensors.cpp` / `AuxSensors.h`).
`Engine_meter.ino` is the original Arduino IDE version.

## Battery telemetry
The firmware samples battery voltage on P0.04 / AIN2 (pin 4), averages 16 ADC
reads, and converts the result to an approximate single-cell LiPo percentage
using a voltage curve. The configured divider ratio is 0.713; verify that the
exact nice!nano module and board wiring use this ratio before relying on the
reported percentage. Voltage-based state of charge is approximate and may be
high while charging or sag under load.

The Nordic UART packet is 10 bytes: the existing 9-byte telemetry packet has
been extended with battery percentage (byte 8, 0-100), and the XOR checksum
is now byte 9 and covers bytes 0-8. The BLE receiver must use this updated
packet length and checksum position to decode the extended packet.

## Board definition
`nicenano` isn't built into PlatformIO, so `boards/nicenano.json` and `boards/nicenano/` (pin variant)
are included and picked up via `board_build.variants_dir = boards` in `platformio.ini`.
Notes from the upstream project: upload may need to be run twice, and you can double-tap reset to enter the bootloader.
