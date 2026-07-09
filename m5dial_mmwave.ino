#include <M5Dial.h>
#include "LD2410Sensor.h"
#include "BluetoothManager.h"

// ============================================================
// CONFIGURATION
// ============================================================
#define SENSOR_RX_PIN 13
//yellow cable
#define SENSOR_TX_PIN 15
//white cable

// ============================================================
// GLOBALS
// ============================================================
LD2410Sensor* sensor = nullptr;
BluetoothManager* bleManager = nullptr;

M5Canvas canvas(&M5Dial.Display);

// Presence tracking
bool wasPresenceDetected = false;
unsigned long lastPresenceTime = 0;
const unsigned long PRESENCE_DEBOUNCE_MS = 1000;  // Debounce presence changes

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== M5DIAL + LD2410C + BLE ===");

  auto cfg = M5.config();
  M5Dial.begin(cfg, true);

  M5Dial.Display.setRotation(0);
  uint16_t w = M5Dial.Display.width();
  uint16_t h = M5Dial.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);

  // Initialize sensor
  sensor = new LD2410Sensor(Serial1, SENSOR_RX_PIN, SENSOR_TX_PIN, 256000);
  sensor->begin();

  // Initialize Bluetooth
  bleManager = new BluetoothManager("M5-mmWave");
  bleManager->begin();

  Serial.println("[INIT] Complete");
}

void updatePresenceDetection() {
  if (!sensor) return;

  bool isPresent = sensor->isPresenceDetected();
  int distance = (int)min(sensor->getMovingDistance(), sensor->getStationaryDistance());
  int maxStrength = max(sensor->getMovingStrength(), sensor->getStationaryStrength());

  // Debounce presence changes
  if (isPresent && !wasPresenceDetected) {
    if (millis() - lastPresenceTime > PRESENCE_DEBOUNCE_MS) {
      Serial.println("[PRESENCE] Detected!");
      bleManager->sendPresenceDetected(distance, maxStrength);
      wasPresenceDetected = true;
      lastPresenceTime = millis();
    }
  } else if (!isPresent && wasPresenceDetected) {
    if (millis() - lastPresenceTime > PRESENCE_DEBOUNCE_MS) {
      Serial.println("[PRESENCE] Cleared!");
      bleManager->sendPresenceCleared();
      wasPresenceDetected = false;
      lastPresenceTime = millis();
    }
  }
}

void drawInterface() {
  canvas.fillSprite(BLACK);

  // Title
  canvas.setCursor(5, 5);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.println("LD2410C + BLE");
  canvas.drawFastHLine(0, 18, canvas.width() - 10, WHITE);

  // BLE Connection Status
  canvas.setTextSize(1);
  canvas.setCursor(5, 24);
  if (bleManager && bleManager->isConnected()) {
    canvas.setTextColor(GREEN);
    canvas.printf("BLE: Connected");
  } else {
    canvas.setTextColor(YELLOW);
    canvas.printf("BLE: Waiting...");
  }

  // Sensor Status
  canvas.setTextSize(1);
  canvas.setCursor(5, 34);
  if (sensor && sensor->hasRecentData()) {
    canvas.setTextColor(BLUE);
    canvas.printf("Sensor: Active");
  } else {
    canvas.setTextColor(RED);
    canvas.printf("Sensor: No Data");
  }

  // Presence Status (large)
  canvas.setTextSize(2);
  canvas.setCursor(5, 52);
  if (sensor) {
    if (sensor->isPresenceDetected()) {
      canvas.setTextColor(GREEN);
      int distance = (int)min(sensor->getMovingDistance(), sensor->getStationaryDistance());
      canvas.printf("PRESENT: %d cm", distance);
    } else {
      canvas.setTextColor(LIGHTGREY);
      canvas.printf("Empty");
    }
  }

  // Detailed readings
  canvas.setTextSize(1);
  canvas.setTextColor(CYAN);
  canvas.setCursor(5, 80);
  if (sensor) {
    canvas.printf("Moving:    %.0f cm (%d%%)", sensor->getMovingDistance(), sensor->getMovingStrength());
    canvas.setCursor(5, 90);
    canvas.printf("Stationary: %.0f cm (%d%%)", sensor->getStationaryDistance(), sensor->getStationaryStrength());
  }

  // Frame counter
  canvas.setTextSize(1);
  canvas.setTextColor(CYAN);
  canvas.setCursor(5, 104);
  if (sensor) {
    canvas.printf("Frames: %d", sensor->getValidFrames());
  }

  canvas.pushSprite(2, 2);
}

void loop() {
  M5Dial.update();

  if (sensor) {
    sensor->update();
  }

  updatePresenceDetection();

  drawInterface();

  delay(10);
}
