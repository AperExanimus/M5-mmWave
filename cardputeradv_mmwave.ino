#include <M5Cardputer.h>
#include <Wire.h>

// ============================================================
// PIN & CONFIGURATION DEFINITIONS
// ============================================================
#define SENSOR_RX_PIN 15   // CardPuter GPIO15 ← Sensor TX
#define SENSOR_TX_PIN 13   // CardPuter GPIO13 ← Sensor RX

// Fixed baud per spec
const int CURRENT_BAUD = 256000;

HardwareSerial ld2410(1);

String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 150; // shorten for more responsive tuning
char lastCapturedKey = 0;

// SENSOR STATUS TRACKING
int byteCount = 0;
bool anyDataReceived = false;
unsigned long dataTimestamp = 0;

// Frame counters
int validFrames = 0;

M5Canvas canvas(&M5Cardputer.Display);

// Detection parameters (runtime-adjustable)
const int MAX_WINDOW = 512;     // hard cap for buffer memory
int personWindow = 128;         // active window size (adjustable with A/S)
int personThreshold = 6;        // detection threshold (adjustable with W/Q)

// Detection buffers/state
uint8_t diffBuf[MAX_WINDOW];
int diffIdx = 0;
int diffCount = 0;
unsigned long diffSum = 0;
int prevByte = -1;
bool personDetected = false;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CARDPUTER + LD2410C - FIXED 256000 BAUD ===");
  
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  
  M5Cardputer.Display.setRotation(1);  // 90° CW - right-side up
  
  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);
  
  Serial.printf("[UART] Initializing at %d baud\n", CURRENT_BAUD);
  ld2410.begin(CURRENT_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(500);
  
  Serial.println("[INIT] Complete");
}

void drawInterface() {
  canvas.fillSprite(BLACK);
  
  // Title
  canvas.setCursor(5, 5);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.println("LD2410C Interactive Scanner");
  canvas.drawFastHLine(0, 18, canvas.width() - 10, WHITE);
  
  // Controls info
  canvas.setCursor(5, 24);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.printf("W/Q: Thresh -/+  |  A/S: Window -/+  | Space: Reset | Esc: Restart");
  
  // Current baud - readable display
  canvas.setTextSize(1);
  canvas.setTextColor(YELLOW);
  canvas.setCursor(5, 38);
  canvas.printf("Baud: %d", CURRENT_BAUD);
  
  // Threshold & Window display
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, 50);
  canvas.printf("Thresh: %d    Window: %d", personThreshold, personWindow);
  
  // Activity indicator (color-coded by activity level)
  canvas.setTextSize(1);
  canvas.setCursor(5, 64);
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
  
  // Person-detected field (prominent)
  canvas.setTextSize(2);
  if (personDetected) {
    canvas.setTextColor(RED);
    canvas.setCursor(5, 86);
    canvas.printf("Person: YES");
  } else {
    canvas.setTextColor(GREEN);
    canvas.setCursor(5, 86);
    canvas.printf("Person: NO ");
  }
  
  // Status line
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, canvas.height() - 14);
  canvas.printf("Status: %s", statusMessage.c_str());
  
  canvas.pushSprite(2, 2);
}

void loop() {
  M5Cardputer.update();
  
  // ===========================================
  // STEP 1: READ ANY AVAILABLE DATA
  // ===========================================
  while (ld2410.available()) {
    byte b = ld2410.read();
    byteCount++;
    anyDataReceived = true;
    dataTimestamp = millis();
    
    // Print hex to serial monitor
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    
    // Newline every 32 bytes for readability
    if (byteCount % 32 == 0) Serial.println("");
    
    // --- Update detection buffers using diffs ---
    if (prevByte >= 0) {
      uint8_t diff = (uint8_t)abs((int)b - prevByte);
      if (personWindow > MAX_WINDOW) personWindow = MAX_WINDOW;
      if (personWindow < 4) personWindow = 4;
      
      if (diffCount < personWindow) {
        // filling buffer
        diffBuf[diffIdx] = diff;
        diffSum += diff;
        diffIdx = (diffIdx + 1) % personWindow;
        diffCount++;
      } else {
        // sliding window: replace oldest (use modulo over current window)
        int replaceIdx = diffIdx % personWindow;
        diffSum -= diffBuf[replaceIdx];
        diffBuf[replaceIdx] = diff;
        diffSum += diff;
        diffIdx = (replaceIdx + 1) % personWindow;
      }
      
      // compute average diff and check threshold
      if (diffCount > 0) {
        unsigned long avg = diffSum / diffCount;
        personDetected = (avg > (unsigned long)personThreshold);
      } else {
        personDetected = false;
      }
    }
    prevByte = b;
  }
  
  // Reset flag after timeout
  if (anyDataReceived && millis() - dataTimestamp > 2000) {
    statusMessage = "No recent data...";
  }
  
  // ===========================================
  // STEP 2: Process keyboard controls
  // ===========================================
  if (M5Cardputer.Keyboard.isChange()) {
    if (M5Cardputer.Keyboard.isPressed()) {
      Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();
      
      for (auto k : status.word) {
        lastCapturedKey = k;
        
        if (millis() - lastKeyTime > KEY_DELAY_MS) {
          // W = decrease threshold (make detection more sensitive)
          if (k == 'w' || k == 'W') {
            if (personThreshold > 0) personThreshold--;
            statusMessage = "Threshold decreased";
          } 
          // Q = increase threshold (less sensitive)
          else if (k == 'q' || k == 'Q') {
            if (personThreshold < 255) personThreshold++;
            statusMessage = "Threshold increased";
          } 
          // A = decrease window (faster, less smoothing)
          else if (k == 'a' || k == 'A') {
            if (personWindow > 4) {
              personWindow = max(4, personWindow / 2); // step down (aggressive)
              // reset buffer to avoid mismatch between old buffer and new window
              resetDetectionBuffer("Window decreased - recalibrated");
            }
          }
          // S = increase window (slower, more smoothing)
          else if (k == 's' || k == 'S') {
            if (personWindow < MAX_WINDOW) {
              personWindow = min(MAX_WINDOW, personWindow * 2); // step up
              resetDetectionBuffer("Window increased - recalibrated");
            }
          }
          // Space = Reset all counters and clear detection buffer (calibrate)
          else if (k == ' ') {
            byteCount = 0;
            validFrames = 0;
            anyDataReceived = false;
            resetDetectionBuffer("Counters & detection reset");
          }
          // Escape = Full restart
          else if (k == '\x1B') {
            esp_restart();
          }
          
          lastKeyTime = millis();
        }
      }
    }
  }
  
  drawInterface();
}

// ============================================================
// HELPER: Reset detection buffer with optional status message
// ============================================================
void resetDetectionBuffer(const char* msg) {
  diffIdx = 0;
  diffCount = 0;
  diffSum = 0;
  prevByte = -1;
  personDetected = false;
  statusMessage = msg;
}

// ============================================================
// NOTE: Baud is fixed; no runtime baud-change functions remain
// ============================================================
