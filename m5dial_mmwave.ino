#include <M5Dial.h>
#include <Wire.h>

// ============================================================
// PIN & CONFIGURATION DEFINITIONS
// ============================================================
#define SENSOR_RX_PIN 13   // M5Dial GPIO13 ← Sensor TX
#define SENSOR_TX_PIN 15   // M5Dial GPIO15 ← Sensor RX
#define FIXED_BAUD 256000  // Fixed baud rate (no switching)

HardwareSerial ld2410(1);

String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 300;

// SENSOR STATUS TRACKING
int byteCount = 0;
bool anyDataReceived = false;
unsigned long dataTimestamp = 0;

// Frame counters
int validFrames = 0;

// DISTANCE TRACKING - RAW
int stationaryDistanceRaw = -1;
int movingDistanceRaw = -1;
int stationaryStrengthRaw = 0;
int movingStrengthRaw = 0;
int targetStateRaw = 0;

// DISTANCE TRACKING - FILTERED (smoothed)
float stationaryDistanceFiltered = -1.0;
float movingDistanceFiltered = -1.0;
int stationaryStrengthFiltered = 0;
int movingStrengthFiltered = 0;
int targetStateFiltered = 0;

// Smoothing parameters (0.0-1.0, lower = more smoothing)
const float SMOOTHING_FACTOR = 0.3;
const int CONFIDENCE_THRESHOLD = 30; // signal strength threshold

// PRESENCE DETECTION & TONE
// Define a target zone (in cm) where you want the tone to trigger
const int TARGET_DISTANCE_CENTER = 100;  // Where your chair is (cm)
const int TARGET_DISTANCE_RANGE = 30;    // ±30 cm around center (so 70-130 cm window)
const int PRESENCE_DISTANCE_MIN = TARGET_DISTANCE_CENTER - TARGET_DISTANCE_RANGE;
const int PRESENCE_DISTANCE_MAX = TARGET_DISTANCE_CENTER + TARGET_DISTANCE_RANGE;

const int HIGH_CONFIDENCE_FRAMES = 10;  // Frames needed to confirm high confidence presence
const int LOSS_CONFIDENCE_FRAMES = 15;  // Frames to wait before losing presence
const unsigned long TONE_INTERVAL_MS = 500; // Time between tone repetitions (ms)

int presenceFrameCount = 0;  // Counter for consecutive high-confidence frames
int lossFrameCount = 0;      // Counter for frames without presence
bool presenceDetected = false;
bool tonePlaying = false;
unsigned long lastToneTime = 0;

// Frame parsing
byte frameBuffer[256];
int frameIndex = 0;
bool frameStarted = false;

M5Canvas canvas(&M5Dial.Display);

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== M5DIAL + LD2410C ===");
  Serial.printf("[CONFIG] Detection zone: %d-%d cm (center=%d, range=±%d)\n", 
                PRESENCE_DISTANCE_MIN, PRESENCE_DISTANCE_MAX, TARGET_DISTANCE_CENTER, TARGET_DISTANCE_RANGE);
  
  auto cfg = M5.config();
  M5Dial.begin(cfg, true);
  
  // M5Dial display setup
  M5Dial.Display.setRotation(0);
  
  uint16_t w = M5Dial.Display.width();
  uint16_t h = M5Dial.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);
  
  Serial.printf("[UART] Initializing at %d baud\n", FIXED_BAUD);
  ld2410.begin(FIXED_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(500);
  
  Serial.println("[INIT] Complete");
}

void applySmoothing() {
  // Apply exponential smoothing to raw values
  // Formula: filtered = (raw * alpha) + (previous * (1 - alpha))
  
  // Stationary distance
  if (stationaryDistanceRaw > 0) {
    if (stationaryDistanceFiltered < 0) {
      stationaryDistanceFiltered = stationaryDistanceRaw;
    } else {
      stationaryDistanceFiltered = (stationaryDistanceRaw * SMOOTHING_FACTOR) + 
                                   (stationaryDistanceFiltered * (1.0 - SMOOTHING_FACTOR));
    }
  }
  
  // Moving distance
  if (movingDistanceRaw > 0) {
    if (movingDistanceFiltered < 0) {
      movingDistanceFiltered = movingDistanceRaw;
    } else {
      movingDistanceFiltered = (movingDistanceRaw * SMOOTHING_FACTOR) + 
                               (movingDistanceFiltered * (1.0 - SMOOTHING_FACTOR));
    }
  }
  
  // Strength values
  movingStrengthFiltered = (int)((movingStrengthRaw * SMOOTHING_FACTOR) + 
                                 (movingStrengthFiltered * (1.0 - SMOOTHING_FACTOR)));
  stationaryStrengthFiltered = (int)((stationaryStrengthRaw * SMOOTHING_FACTOR) + 
                                     (stationaryStrengthFiltered * (1.0 - SMOOTHING_FACTOR)));
  
  // Update target state based on filtered strengths
  bool hasMoving = (movingDistanceFiltered > 0 && movingStrengthFiltered > CONFIDENCE_THRESHOLD);
  bool hasStationary = (stationaryDistanceFiltered > 0 && stationaryStrengthFiltered > CONFIDENCE_THRESHOLD);
  
  if (hasMoving && hasStationary) {
    targetStateFiltered = 3;  // Both
  } else if (hasMoving) {
    targetStateFiltered = 1;  // Moving only
  } else if (hasStationary) {
    targetStateFiltered = 2;  // Stationary only
  } else {
    targetStateFiltered = 0;  // No target
  }
}

void checkPresence() {
  // Check if someone is in the target zone with high confidence
  float closestDistance = -1.0;
  int maxStrength = 0;
  
  // Determine closest target and max strength
  if (movingDistanceFiltered > 0 && stationaryDistanceFiltered > 0) {
    closestDistance = min(movingDistanceFiltered, stationaryDistanceFiltered);
    maxStrength = max(movingStrengthFiltered, stationaryStrengthFiltered);
  } else if (movingDistanceFiltered > 0) {
    closestDistance = movingDistanceFiltered;
    maxStrength = movingStrengthFiltered;
  } else if (stationaryDistanceFiltered > 0) {
    closestDistance = stationaryDistanceFiltered;
    maxStrength = stationaryStrengthFiltered;
  }
  
  // HIGH CONFIDENCE PRESENCE: someone is IN THE TARGET ZONE with strong signal
  bool highConfidencePresence = (closestDistance >= PRESENCE_DISTANCE_MIN && 
                                 closestDistance <= PRESENCE_DISTANCE_MAX && 
                                 maxStrength > CONFIDENCE_THRESHOLD);
  
  if (highConfidencePresence) {
    presenceFrameCount++;
    lossFrameCount = 0;  // Reset loss counter when we detect presence
    
    // Once we've seen high confidence for enough frames, set flag to play tone
    if (presenceFrameCount >= HIGH_CONFIDENCE_FRAMES && !presenceDetected) {
      presenceDetected = true;
      tonePlaying = true;
      lastToneTime = 0;  // Force immediate first tone
      Serial.println("[PRESENCE] HIGH CONFIDENCE PERSON DETECTED IN TARGET ZONE - Starting tone loop!");
    }
  } else {
    // Increment loss counter when out of zone
    lossFrameCount++;
    
    // If we haven't seen presence for enough frames, stop detection
    if (lossFrameCount >= LOSS_CONFIDENCE_FRAMES && presenceDetected) {
      presenceDetected = false;
      tonePlaying = false;
      presenceFrameCount = 0;
      Serial.println("[PRESENCE] Person left target zone - Stopping tone loop");
    }
  }
}

void playRepeatingTone() {
  // Play tone repeatedly while presence is detected in target zone
  if (presenceDetected && tonePlaying && (millis() - lastToneTime > TONE_INTERVAL_MS)) {
    M5Dial.Speaker.tone(1000, 100);  // 1000 Hz, 100 ms duration
    Serial.printf("[TONE] Playing (Distance: %.1f cm, Strength: %d%%)\n", 
                  min(movingDistanceFiltered, stationaryDistanceFiltered), 
                  max(movingStrengthFiltered, stationaryStrengthFiltered));
    lastToneTime = millis();
  }
  
  // Stop tone if presence is lost
  if (!presenceDetected && tonePlaying) {
    tonePlaying = false;
    M5Dial.Speaker.end();
    Serial.println("[TONE] Stopped");
  }
}

void parseFrame() {
  // LD2410 frame format (verified with actual data):
  // [0-3]    Header: 0xF4 0xF3 0xF2 0xF1
  // [4-5]    Data length (little-endian): LSB MSB
  // [6-7]    Frame type: should be 0x02 0xAA for data frame
  // [8]      Target state (0=none, 1=moving, 2=stationary, 3=both)
  // [9-10]   Moving distance (little-endian, in cm)
  // [11]     Moving signal strength
  // [12-13]  Stationary distance (little-endian, in cm)
  // [14]     Stationary signal strength
  // [15+]    Additional data (light level, output state, etc.)
  // [len-4]  Footer: 0xF8 0xF7 0xF6 0xF5
  
  if (frameIndex < 20) return;  // Minimum valid frame size
  
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
  
  // Check frame type for data frame (0xAA02 or 0x02AA)
  if ((frameBuffer[6] != 0x02 || frameBuffer[7] != 0xAA) &&
      (frameBuffer[6] != 0xAA || frameBuffer[7] != 0x02)) {
    return;
  }
  
  // Extract target state and distances
  targetStateRaw = frameBuffer[8];
  
  // Try two possible byte orderings to find correct parsing
  // Option 1: Standard ordering
  int movDist1 = frameBuffer[9] | (frameBuffer[10] << 8);
  int statDist1 = frameBuffer[12] | (frameBuffer[13] << 8);
  
  // Store raw values
  movingDistanceRaw = (movDist1 > 0 && movDist1 < 1000) ? movDist1 : -1;
  movingStrengthRaw = frameBuffer[11];
  stationaryDistanceRaw = (statDist1 > 0 && statDist1 < 1000) ? statDist1 : -1;
  stationaryStrengthRaw = frameBuffer[14];
  
  // Apply smoothing filter
  applySmoothing();
  
  // Check for presence
  checkPresence();
  
  validFrames++;
  dataTimestamp = millis();
  anyDataReceived = true;
  
  float closestDist = -1.0;
  if (movingDistanceFiltered > 0 && stationaryDistanceFiltered > 0) {
    closestDist = min(movingDistanceFiltered, stationaryDistanceFiltered);
  } else if (movingDistanceFiltered > 0) {
    closestDist = movingDistanceFiltered;
  } else if (stationaryDistanceFiltered > 0) {
    closestDist = stationaryDistanceFiltered;
  }
  
  bool inZone = (closestDist >= PRESENCE_DISTANCE_MIN && closestDist <= PRESENCE_DISTANCE_MAX);
  
  Serial.printf("[FRAME %d] Closest: %.0f cm [%s] | Strength: %d%% | Zone: %s | Presence: %s (%d/%d)\n", 
                validFrames, closestDist, inZone ? "IN" : "OUT",
                max(movingStrengthFiltered, stationaryStrengthFiltered),
                (closestDist >= PRESENCE_DISTANCE_MIN && closestDist <= PRESENCE_DISTANCE_MAX) ? "YES" : "NO",
                presenceDetected ? "DETECTED" : "WAITING", presenceFrameCount, HIGH_CONFIDENCE_FRAMES);
}

void drawInterface() {
  canvas.fillSprite(BLACK);
  
  // Title
  canvas.setCursor(5, 5);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.println("LD2410C Scanner");
  canvas.drawFastHLine(0, 18, canvas.width() - 10, WHITE);
  
  // Target zone info
  canvas.setCursor(5, 24);
  canvas.setTextSize(1);
  canvas.setTextColor(YELLOW);
  canvas.printf("Zone: %d-%d cm", PRESENCE_DISTANCE_MIN, PRESENCE_DISTANCE_MAX);
  
  // Fixed baud display
  canvas.setTextSize(1);
  canvas.setTextColor(YELLOW);
  canvas.setCursor(5, 38);
  canvas.printf("Baud: %d Hz", FIXED_BAUD);
  
  // Activity indicator
  canvas.setTextSize(1);
  canvas.setCursor(5, 52);
  if (anyDataReceived && millis() - dataTimestamp < 1000) {
    canvas.setTextColor(BLUE);
    canvas.printf("ACTIVE");
  } else if (anyDataReceived && millis() - dataTimestamp < 2000) {
    canvas.setTextColor(GREEN);
    canvas.printf("Some data...");
  } else {
    canvas.setTextColor(RED);
    canvas.printf("NO DATA");
  }
  
  // DISTANCE DISPLAY (FILTERED - main display)
  canvas.setTextSize(2);
  canvas.setCursor(5, 68);
  
  float closestDist = -1.0;
  if (movingDistanceFiltered > 0 && stationaryDistanceFiltered > 0) {
    closestDist = min(movingDistanceFiltered, stationaryDistanceFiltered);
  } else if (movingDistanceFiltered > 0) {
    closestDist = movingDistanceFiltered;
  } else if (stationaryDistanceFiltered > 0) {
    closestDist = stationaryDistanceFiltered;
  }
  
  if (targetStateFiltered == 0) {
    canvas.setTextColor(RED);
    canvas.printf("No Target");
  } else {
    bool inZone = (closestDist >= PRESENCE_DISTANCE_MIN && closestDist <= PRESENCE_DISTANCE_MAX);
    if (inZone) {
      canvas.setTextColor(GREEN);
      canvas.printf("IN ZONE: %.0f cm", closestDist);
    } else {
      canvas.setTextColor(DARKGREY);
      canvas.printf("Out: %.0f cm", closestDist);
    }
  }
  
  // Signal strengths (filtered)
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, 90);
  canvas.printf("Strength: %d%%", max(movingStrengthFiltered, stationaryStrengthFiltered));
  
  // Presence status with tone indicator
  canvas.setTextSize(1);
  canvas.setCursor(5, 104);
  if (presenceDetected && tonePlaying) {
    canvas.setTextColor(GREEN);
    canvas.printf("PERSON IN ZONE! [TONE ON]");
  } else if (presenceDetected) {
    canvas.setTextColor(YELLOW);
    canvas.printf("Detecting... (%d/%d)", presenceFrameCount, HIGH_CONFIDENCE_FRAMES);
  } else if (lossFrameCount > 0 && lossFrameCount < LOSS_CONFIDENCE_FRAMES) {
    canvas.setTextColor(YELLOW);
    canvas.printf("Person leaving... (%d/%d)", lossFrameCount, LOSS_CONFIDENCE_FRAMES);
  } else {
    canvas.setTextColor(RED);
    canvas.printf("No one in zone");
  }
  
  // Frame counter
  canvas.setTextSize(1);
  canvas.setTextColor(CYAN);
  canvas.setCursor(5, 118);
  canvas.printf("Frames: %d", validFrames);
  
  canvas.pushSprite(2, 2);
}

void loop() {
  M5Dial.update();
  
  // ===========================================
  //  READ ANY AVAILABLE DATA
  // ===========================================
  while (ld2410.available()) {
    byte b = ld2410.read();
    byteCount++;
    
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
  
  // Play repeating tone while presence is detected in target zone
  playRepeatingTone();
  
  // Reset after timeout
  if (anyDataReceived && millis() - dataTimestamp > 2000) {
    statusMessage = "No recent data...";
    targetStateFiltered = 0;
    movingDistanceFiltered = -1.0;
    stationaryDistanceFiltered = -1.0;
    presenceDetected = false;
    tonePlaying = false;
    presenceFrameCount = 0;
    lossFrameCount = 0;
  }
  
  drawInterface();
}
