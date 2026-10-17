# Engine_meter
nRF52840 (nice!nano) engine meter -- RPM + EGT/CHT over BLE (Nordic UART) to the paraglide_vario_ESP32.

## Build in VS Code (PlatformIO)
1. Install the **PlatformIO IDE** extension.
2. File > Open Folder... > select this `Engine_meter` folder.
3. Build: PlatformIO sidebar > `nicenano` > General > Build (or Ctrl+Alt+B).
4. Start PlatformIO Upload; PlatformIO will try to reset the nice!nano into its
   bootloader automatically. If the upload hangs or fails, double-tap the reset
   button to enter the bootloader manually and retry. The bootloader appears as
   a separate USB serial COM port (`VID_239A&PID_00B3`); upload must target that
   port, not the running firmware's COM port.
5. To monitor, open the running firmware's COM port at 115200 baud. Start the
   monitor, then reset or reconnect the board to catch the startup message.
   Firmware waits up to 1.5 seconds for USB serial at boot; it continues running
   if no monitor is connected and prints sensor readings every 200 ms.

Firmware source is in `src/` (`main.cpp` and `AuxSensors.cpp` / `AuxSensors.h`).
`Engine_meter.ino` is the original Arduino IDE version.

## BLE NMEA telemetry
The nice!nano sends ASCII NMEA-style sentences through Nordic UART Service
(NUS), with an XOR checksum over the bytes between `$` and `*` and CRLF line
endings. Both Gaggle and the ESP32 vario consume this same text format:

- `$EMS,<pressure>,<fuelFlow>,<fuelRemaining>,<fuelUsed>,<ambientTemp>,<CHT>,<EGT>,<waterTemp>,<RPM>*HH\r\n`
  has nine fields after `EMS`, in that order. Pressure is `999999`; fuel flow,
  remaining, and used are `9999`; ambient and water temperatures are `99`.
  CHT and EGT are rounded degrees C x 100, clamped to -300..1500 C; `99`
  means unavailable (fault or non-finite reading). RPM is rounded RPM x 100;
  `0` is a valid stopped-engine reading. Raw RPM `9999` is reserved as
  unavailable, so a computed value of exactly `9999` is sent as `10000`.
- `$PEMBAT,<percent>*HH\r\n` reports battery percentage from 0 to 100.
  Sending it is controlled by `SEND_PEMBAT` in `src/main.cpp` (set it to `0`
  if Gaggle objects to the proprietary sentence).

Battery percentage is estimated from the nice!nano's internal nRF52840
VDDHDIV5 ADC input and a single-cell LiPo voltage curve. Voltage-based state
of charge is approximate and may read high while charging or sag under load.

For Gaggle, add the meter through **Profile > Equipment > Bluetooth Devices**.
Gaggle needs to receive a real supported reading within about 35 seconds;
the `EMS` sentence supplies supported engine readings such as CHT, EGT, and
RPM.

### RPM filtering
RPM is calculated from pulse-count snapshots taken every 200 ms and averaged
over a gated 5-second window. A 1-second fast-attack window is enabled by
default and is used when it differs from the 5-second average by more than
10%, so real RPM changes are followed more quickly. The displayed value is
rounded to the nearest 10 RPM with 7 RPM hysteresis to reduce flicker;
`$EMS` carries that rounded RPM multiplied by 100. If no accepted pulse
arrives within the 1.5-second stop timeout, RPM is reported as zero
immediately and the averaging history is cleared so no decaying RPM remains.

## Board definition
`nicenano` isn't built into PlatformIO, so `boards/nicenano.json` and `boards/nicenano/` (pin variant)
are included and picked up via `board_build.variants_dir = boards` in `platformio.ini`.
