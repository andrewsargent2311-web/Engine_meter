# Engine_meter
nRF52840 (nice!nano) engine meter -- RPM + EGT/CHT over BLE (Nordic UART) to the paraglide_vario_ESP32.

## Build in VS Code (PlatformIO)
1. Install the **PlatformIO IDE** extension.
2. File > Open Folder... > select this `Engine_meter` folder.
3. Build: PlatformIO sidebar > `nicenano` > General > Build (or Ctrl+Alt+B).
4. Upload / Monitor from the same menu (serial monitor is 115200 baud).

Firmware source is `src/main.cpp`. `Engine_meter.ino` is the original Arduino IDE version.

## Board definition
`nicenano` isn't built into PlatformIO, so `boards/nicenano.json` and `boards/nicenano/` (pin variant)
are included and picked up via `board_build.variants_dir = boards` in `platformio.ini`.
Notes from the upstream project: upload may need to be run twice, and you can double-tap reset to enter the bootloader.
