#ifndef LD2410_SENSOR_H
#define LD2410_SENSOR_H

#include <HardwareSerial.h>

class LD2410Sensor {
public:
  // Target state enums
  enum TargetState {
    NO_TARGET = 0,
    MOVING_ONLY = 1,
    STATIONARY_ONLY = 2,
    BOTH = 3
  };

  // Constructor
  LD2410Sensor(HardwareSerial& serial, uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 256000);

  // Initialization
  bool begin();

  // Main update - call this in loop()
  void update();

  // Getters for filtered data
  int getTargetState() const { return targetStateFiltered; }
  float getMovingDistance() const { return movingDistanceFiltered; }
  float getStationaryDistance() const { return stationaryDistanceFiltered; }
  int getMovingStrength() const { return movingStrengthFiltered; }
  int getStationaryStrength() const { return stationaryStrengthFiltered; }
  int getValidFrames() const { return validFrames; }
  bool hasRecentData() const;
  bool isPresenceDetected() const;  // Returns true if confident target detected

  // Config
  void setConfidenceThreshold(int threshold) { confidenceThreshold = threshold; }
  void setSmoothingFactor(float factor) { smoothingFactor = factor; }

private:
  HardwareSerial& serialPort;
  uint8_t rxPin, txPin;
  uint32_t baudRate;

  // Raw values
  int stationaryDistanceRaw = -1;
  int movingDistanceRaw = -1;
  int stationaryStrengthRaw = 0;
  int movingStrengthRaw = 0;
  int targetStateRaw = 0;

  // Filtered values
  float stationaryDistanceFiltered = -1.0;
  float movingDistanceFiltered = -1.0;
  int stationaryStrengthFiltered = 0;
  int movingStrengthFiltered = 0;
  int targetStateFiltered = 0;

  // Frame parsing
  uint8_t frameBuffer[256];
  int frameIndex = 0;
  bool frameStarted = false;

  // Statistics
  int validFrames = 0;
  unsigned long dataTimestamp = 0;

  // Config parameters
  float smoothingFactor = 0.3;
  int confidenceThreshold = 30;
  const int DISTANCE_MAX = 1000;  // cm

  // Private methods
  void parseFrame();
  void applySmoothing();
};

#endif