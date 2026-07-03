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
  
  // Activity indicator (color-coded by activity level)
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
  
  // Status line
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, 66);
  canvas.printf("Status: %s", statusMessage.c_str());
  
  // Byte counter
  canvas.setTextSize(1);
  canvas.setTextColor(CYAN);
  canvas.setCursor(5, 80);
  canvas.printf("Bytes: %d | Frames: %d", byteCount, validFrames);
  
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
  
  drawInterface();
}
