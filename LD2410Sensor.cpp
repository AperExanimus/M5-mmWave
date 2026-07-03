#include "LD2410Sensor.h"

LD2410Sensor::LD2410Sensor(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
    : serialPort(serial), rxPin(rxPin), txPin(txPin), baudRate(baudRate) {
}

bool LD2410Sensor::begin() {
  Serial.printf("[LD2410] Initializing at %d baud\n", baudRate);
  serialPort.begin(baudRate, SERIAL_8N1, rxPin, txPin);
  delay(500);
  Serial.println("[LD2410] Ready");
  return true;
}

void LD2410Sensor::update() {
  // Read available data and parse frames
  while (serialPort.available()) {
    uint8_t b = serialPort.read();

    // Look for frame start marker
    if (!frameStarted && b == 0xF4) {
      frameStarted = true;
      frameIndex = 0;
    }

    if (frameStarted) {
      frameBuffer[frameIndex++] = b;

      // Look for frame end marker (0xF5 0xF6 0xF7 0xF8)
      if (frameIndex >= 4 &&
          frameBuffer[frameIndex - 4] == 0xF8 &&
          frameBuffer[frameIndex - 3] == 0xF7 &&
          frameBuffer[frameIndex - 2] == 0xF6 &&
          frameBuffer[frameIndex - 1] == 0xF5) {

        parseFrame();
        frameStarted = false;
        frameIndex = 0;
      }

      // Prevent buffer overflow
      if (frameIndex >= 256) {
        frameStarted = false;
        frameIndex = 0;
      }
    }
  }

  // Reset after timeout
  if (dataTimestamp > 0 && millis() - dataTimestamp > 2000) {
    targetStateFiltered = 0;
    movingDistanceFiltered = -1.0;
    stationaryDistanceFiltered = -1.0;
  }
}

void LD2410Sensor::parseFrame() {
  if (frameIndex < 20) return;

  // Check header
  if (frameBuffer[0] != 0xF4 || frameBuffer[1] != 0xF3 ||
      frameBuffer[2] != 0xF2 || frameBuffer[3] != 0xF1) {
    return;
  }

  // Check footer
  if (frameBuffer[frameIndex - 4] != 0xF8 ||
      frameBuffer[frameIndex - 3] != 0xF7 ||
      frameBuffer[frameIndex - 2] != 0xF6 ||
      frameBuffer[frameIndex - 1] != 0xF5) {
    return;
  }

  // Check frame type for data frame
  if ((frameBuffer[6] != 0x02 || frameBuffer[7] != 0xAA) &&
      (frameBuffer[6] != 0xAA || frameBuffer[7] != 0x02)) {
    return;
  }

  // Extract target state and distances
  targetStateRaw = frameBuffer[8];

  // Parse moving distance (little-endian)
  int movDist = frameBuffer[9] | (frameBuffer[10] << 8);
  movingDistanceRaw = (movDist > 0 && movDist < DISTANCE_MAX) ? movDist : -1;
  movingStrengthRaw = frameBuffer[11];

  // Parse stationary distance (little-endian)
  int statDist = frameBuffer[12] | (frameBuffer[13] << 8);
  stationaryDistanceRaw = (statDist > 0 && statDist < DISTANCE_MAX) ? statDist : -1;
  stationaryStrengthRaw = frameBuffer[14];

  // Apply smoothing filter
  applySmoothing();

  validFrames++;
  dataTimestamp = millis();

  Serial.printf("[LD2410 Frame %d] Raw: M=%3dcm(%3d%%) S=%3dcm(%3d%%) | Filtered: M=%.1f S=%.1f\n",
                validFrames,
                (movingDistanceRaw > 0 ? movingDistanceRaw : 0), movingStrengthRaw,
                (stationaryDistanceRaw > 0 ? stationaryDistanceRaw : 0), stationaryStrengthRaw,
                movingDistanceFiltered, stationaryDistanceFiltered);
}

void LD2410Sensor::applySmoothing() {
  // Stationary distance
  if (stationaryDistanceRaw > 0) {
    if (stationaryDistanceFiltered < 0) {
      stationaryDistanceFiltered = stationaryDistanceRaw;
    } else {
      stationaryDistanceFiltered = (stationaryDistanceRaw * smoothingFactor) +
                                   (stationaryDistanceFiltered * (1.0 - smoothingFactor));
    }
  }

  // Moving distance
  if (movingDistanceRaw > 0) {
    if (movingDistanceFiltered < 0) {
      movingDistanceFiltered = movingDistanceRaw;
    } else {
      movingDistanceFiltered = (movingDistanceRaw * smoothingFactor) +
                               (movingDistanceFiltered * (1.0 - smoothingFactor));
    }
  }

  // Strength values
  movingStrengthFiltered = (int)((movingStrengthRaw * smoothingFactor) +
                                 (movingStrengthFiltered * (1.0 - smoothingFactor)));
  stationaryStrengthFiltered = (int)((stationaryStrengthRaw * smoothingFactor) +
                                     (stationaryStrengthFiltered * (1.0 - smoothingFactor)));

  // Update target state based on filtered strengths
  bool hasMoving = (movingDistanceFiltered > 0 && movingStrengthFiltered > confidenceThreshold);
  bool hasStationary = (stationaryDistanceFiltered > 0 && stationaryStrengthFiltered > confidenceThreshold);

  if (hasMoving && hasStationary) {
    targetStateFiltered = BOTH;
  } else if (hasMoving) {
    targetStateFiltered = MOVING_ONLY;
  } else if (hasStationary) {
    targetStateFiltered = STATIONARY_ONLY;
  } else {
    targetStateFiltered = NO_TARGET;
  }
}

bool LD2410Sensor::hasRecentData() const {
  return dataTimestamp > 0 && (millis() - dataTimestamp < 2000);
}

bool LD2410Sensor::isPresenceDetected() const {
  return (targetStateFiltered != NO_TARGET) &&
         (movingStrengthFiltered > confidenceThreshold || stationaryStrengthFiltered > confidenceThreshold);
}