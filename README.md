# M5-mmWave

Firmware to connect M5-series ESP32 devices (M5Dial, M5Cardputer) to an LD2410 mmWave sensor and relay presence events over BLE.

## Key features
- Reads LD2410C mmWave frames and extracts moving/stationary target distance + strength.
- Simple smoothing and confidence thresholds for robust presence detection.
- BLE peripheral on the sensor device that notifies presence events.
- BLE client example (Cardputer) that scans for notifications and shows incoming presence events.

## Supported hardware
- M5Dial (display example): m5dial_mmwave.ino
- M5Cardputer (BLE client UI): bluetooth_receiver.ino
- M5Cardputer example / alternate sketches: cardputeradv_mmwave.ino
- LD2410 / LD2410C mmWave sensor

## Quick wiring
- LD2410 serial pins -> ESP32 Serial1 pins used in sketches:
  - Sensor TX -> board RX (SENSOR_RX_PIN = 13)
  - Sensor RX -> board TX (SENSOR_TX_PIN = 15)
- Sensor baud used in code: 256000 (Serial1)

(Verify your module pinout and power requirements before connecting.)

## BLE details
- Service UUID: 12345678-1234-5678-1234-56789abcdef0
- Presence characteristic (notify): 12345678-1234-5678-1234-56789abcdef1
- Command characteristic (write/read): 12345678-1234-5678-1234-56789abcdef2
- Status characteristic (read/notify): 12345678-1234-5678-1234-56789abcdef3

Presence notifications use plain-text messages:
- PRESENCE_DETECTED:<distance>cm:<confidence>%
- PRESENCE_CLEARED

## Quick start (Arduino / PlatformIO)
1. Install board support for ESP32 and required libraries:
   - M5Dial / M5Cardputer library (M5Stack family)
   - ESP32 BLE libraries (esp32 BLEDevice / BLEServer)
2. Open `m5dial_mmwave.ino` (for M5Dial + LD2410) or `bluetooth_receiver.ino` (BLE client) in Arduino IDE or PlatformIO.
3. Select the correct target board (M5Dial / M5Cardputer) and upload.
4. Power the LD2410 and connect its TX/RX to the pins defined in the sketch (13/15 by default).
5. Observe serial logs at 115200 for debug and BLE advertising/connection state.

## Configuration
- LD2410Sensor options (in code):
  - smoothingFactor (default 0.3)
  - confidenceThreshold (default 30)
- Change pins, device name, or thresholds directly in the sketches or in `LD2410Sensor` / `BluetoothManager` usage.

## Files of interest
- LD2410Sensor.h / LD2410Sensor.cpp — mmWave frame parsing, smoothing, presence logic
- BluetoothManager.h / BluetoothManager.cpp — BLE peripheral implementation and notifications
- m5dial_mmwave.ino — M5Dial firmware that reads sensor and advertises presence
- bluetooth_receiver.ino — BLE client example that listens for presence notifications
- cardputeradv_mmwave.ino — alternate Cardputer example

## License
MIT — see LICENSE.

## Contributing
PRs and issues welcome. Good first tasks:
- Add PlatformIO config / example platformio.ini
- Add wiring diagrams and photos
- Add configurable settings via menu/UI

## Questions you might want to answer next
- Which board/board definitions and library versions did you test with (exact M5 library / ESP32 core)?
- Do you want packaged PlatformIO examples or a one-click Arduino Library?
- Would you like a simple JSON/CBOR payload for BLE notifications instead of plain text?
