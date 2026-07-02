#include <M5Cardputer.h>
#include <Wire.h>

// ============================================================
// PIN & CONFIGURATION DEFINITIONS
// ============================================================
#define SENSOR_RX_PIN 15   // CardPuter GPIO15 ← Sensor TX
#define SENSOR_TX_PIN 13   // CardPuter GPIO13 ← Sensor RX

// FLEXIBLE BAUD RATE LIST - Edit this array to add/remove rates
int baudRates[] = {9600, 19200, 38400, 57600, 115200, 230400, 256000};
int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);
int currentBaudIdx = 0;
int CURRENT_BAUD = baudRates[currentBaudIdx];

HardwareSerial ld2410(1);

String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 300;
char lastCapturedKey = 0;

// SENSOR STATUS TRACKING
int byteCount = 0;
bool anyDataReceived = false;
unsigned long dataTimestamp = 0;

// Frame counters
int validFrames = 0;

M5Canvas canvas(&M5Cardputer.Display);

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CARDPUTER + LD2410C - FLEXIBLE BAUD ===");
  
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  
  M5Cardputer.Display.setRotation(1);
  
  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setRotation(1);
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
  canvas.setCursor(5, 8);
  canvas.println("LD2410C - Flexible Baud Mode");
  canvas.drawFastHLine(0, 22, 280, WHITE);
  
  // Controls info
  canvas.setCursor(5, 30);
  canvas.setTextSize(0.8);
  canvas.printf("W/Q: Change | Space: Reset | Esc: Restart");
  canvas.setTextSize(1);
  
  // Baud rate box
  canvas.fillRect(5, 45, 270, 45, DARKGREY);
  canvas.setTextColor(YELLOW);
  canvas.setTextSize(1);
  canvas.setCursor(15, 55);
  canvas.printf("Index: %d/%d", currentBaudIdx, numBaudRates - 1);
  
  canvas.setTextColor(CYAN);
  canvas.setTextSize(2);
  canvas.setCursor(15, 75);
  canvas.printf("%d Hz", CURRENT_BAUD);
  
  // Activity indicator (color-coded by activity level)
  canvas.setCursor(5, 105);
  if (anyDataReceived && millis() - dataTimestamp < 1000) {
    canvas.setTextColor(BLUE);
    canvas.printf("ACTIVE (%d bytes/sec)", byteCount);
  } else if (anyDataReceived && millis() - dataTimestamp < 2000) {
    canvas.setTextColor(GREEN);
    canvas.printf("Some data...");
  } else {
    canvas.setTextColor(RED);
    canvas.printf("NO DATA DETECTED");
  }
  
  // Byte pattern hint (shows first few bytes for quick reference)
  canvas.setCursor(5, 125);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(0.7);
  canvas.printf("Tip: Look for non-zero variety in Serial Monitor");
  
  // Status line
  canvas.setCursor(5, 145);
  canvas.setTextSize(1);
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
          // W = Next higher baud rate
          if (k == 'w' || k == 'W') {
            currentBaudIdx++;
            if (currentBaudIdx >= numBaudRates) currentBaudIdx = 0;
            changeBaudRate();
            statusMessage = "↑ Increased baud";
          } 
          // Q = Previous lower baud rate  
          else if (k == 'q' || k == 'Q') {
            currentBaudIdx--;
            if (currentBaudIdx < 0) currentBaudIdx = numBaudRates - 1;
            changeBaudRate();
            statusMessage = "↓ Decreased baud";
          } 
          // Space = Reset all counters
          else if (k == ' ') {
            byteCount = 0;
            validFrames = 0;
            anyDataReceived = false;
            statusMessage = "Counters reset";
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
// HELPER FUNCTION: Change Baud Rate
// ============================================================
void changeBaudRate() {
  int newBaud = baudRates[currentBaudIdx];
  CURRENT_BAUD = newBaud;
  
  Serial.printf("\n>>> CHANGING TO %d BAUD <<<\n\n", CURRENT_BAUD);
  
  ld2410.end();
  delay(200);
  ld2410.begin(CURRENT_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(500);
  
  Serial.printf("[NEW BAUD] Ready at %d\n", CURRENT_BAUD);
  byteCount = 0;
  anyDataReceived = false;
}
