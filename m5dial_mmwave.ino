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

// DISTANCE TRACKING
int stationaryDistance = -1;  // -1 means no target
int movingDistance = -1;
int stationaryStrength = 0;
int movingStrength = 0;
int targetState = 0;          // 0: no target, 1: moving, 2: stationary, 3: both

// Frame parsing
byte frameBuffer[256];
int frameIndex = 0;
bool frameStarted = false;

M5Canvas canvas(&M5Dial.Display);

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== M5DIAL + LD2410C ===");
  
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

void parseFrame() {
  // LD2410 frame format (verified with actual data):
  // [0-3]    Header: 0xF4 0xF3 0xF2 0xF1
  // [4-5]    Data length (little-endian): LSB MSB
  // [6-7]    Frame type: should be 0xAA 0x02 for data frame
  // [8]      Target state (0=none, 1=moving, 2=stationary, 3=both)
  // [9-10]   Moving distance (little-endian, in cm)
  // [11]     Moving signal strength
  // [12-13]  Stationary distance (little-endian, in cm)
  // [14]     Stationary signal strength
  // [15+]    Additional data
  // [len-4]  Footer: 0xF8 0xF7 0xF6 0xF5
  
  if (frameIndex < 20) return;  // Minimum valid frame size
  
  // Check header
  if (frameBuffer[0] != 0xF4 || frameBuffer[1] != 0xF3 || 
      frameBuffer[2] != 0xF2 || frameBuffer[3] != 0xF1) {
    Serial.println("[PARSE] Invalid header");
    return;
  }
  
  // Check footer
  if (frameBuffer[frameIndex - 4] != 0xF8 || 
      frameBuffer[frameIndex - 3] != 0xF7 ||
      frameBuffer[frameIndex - 2] != 0xF6 || 
      frameBuffer[frameIndex - 1] != 0xF5) {
    Serial.println("[PARSE] Invalid footer");
    return;
  }
  
  // Get data length
  int dataLen = frameBuffer[4] | (frameBuffer[5] << 8);
  
  // Check frame type for data frame (0xAA02)
  if (frameBuffer[6] != 0x02 || frameBuffer[7] != 0xAA) {
    Serial.printf("[PARSE] Skipping non-data frame type: %02X %02X\n", frameBuffer[6], frameBuffer[7]);
    return;
  }
  
  // Extract target state and distances
  targetState = frameBuffer[8];
  
  // Moving distance (little-endian)
  int movDist = frameBuffer[9] | (frameBuffer[10] << 8);
  movingDistance = (movDist > 0) ? movDist : -1;
  movingStrength = frameBuffer[11];
  
  // Stationary distance (little-endian)
  int statDist = frameBuffer[12] | (frameBuffer[13] << 8);
  stationaryDistance = (statDist > 0) ? statDist : -1;
  stationaryStrength = frameBuffer[14];
  
  validFrames++;
  dataTimestamp = millis();
  anyDataReceived = true;
  
  Serial.printf("[FRAME %d] State: %d | Moving: %d cm (%d%%) | Static: %d cm (%d%%)\n", 
                validFrames, targetState, movingDistance, movingStrength, 
                stationaryDistance, stationaryStrength);
}

void drawInterface() {
  canvas.fillSprite(BLACK);
  
  // Title
  canvas.setCursor(5, 5);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.println("LD2410C Scanner");
  canvas.drawFastHLine(0, 18, canvas.width() - 10, WHITE);
  
  // Controls info
  canvas.setCursor(5, 24);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.printf("Enc: Reset | Btn: Restart");
  
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
    canvas.printf("ACTIVE (%d b/s)", byteCount);
  } else if (anyDataReceived && millis() - dataTimestamp < 2000) {
    canvas.setTextColor(GREEN);
    canvas.printf("Some data...");
  } else {
    canvas.setTextColor(RED);
    canvas.printf("NO DATA DETECTED");
  }
  
  // DISTANCE DISPLAY (primary)
  canvas.setTextSize(2);
  canvas.setTextColor(CYAN);
  canvas.setCursor(5, 68);
  
  if (targetState == 0) {
    canvas.setTextColor(RED);
    canvas.printf("No Target");
  } else if (targetState == 1) {
    canvas.setTextColor(GREEN);
    canvas.printf("Moving: %d cm", movingDistance);
  } else if (targetState == 2) {
    canvas.setTextColor(YELLOW);
    canvas.printf("Static: %d cm", stationaryDistance);
  } else if (targetState == 3) {
    canvas.setTextColor(CYAN);
    canvas.printf("M:%dcm S:%dcm", movingDistance, stationaryDistance);
  }
  
  // Signal strengths
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, 90);
  canvas.printf("Strength: M:%d%% S:%d%%", movingStrength, stationaryStrength);
  
  // Status line
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, 104);
  canvas.printf("Status: %s", statusMessage.c_str());
  
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
  // STEP 1: READ ANY AVAILABLE DATA
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
  
  // Reset flag after timeout
  if (anyDataReceived && millis() - dataTimestamp > 2000) {
    statusMessage = "No recent data...";
    targetState = 0;
    movingDistance = -1;
    stationaryDistance = -1;
  }
  
  drawInterface();
}
