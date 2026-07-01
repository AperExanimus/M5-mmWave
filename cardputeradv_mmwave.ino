#include <M5Cardputer.h>
#include <Wire.h>

#define SENSOR_RX_PIN 4   
#define SENSOR_TX_PIN 3   
    
int baudRates[] = {9600, 19200, 38400, 57600, 115200, 230400, 256000};
int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);
int currentBaudIdx = 0;

HardwareSerial ld2410(1);

unsigned long startTime = 0;
int byteCount = 0;
bool anyDataReceived = false;
static unsigned long lastKeyTime = 0;
static const unsigned long KEY_DEBOUNCE_MS = 300;

void setup() {
  M5.begin();
  
  Serial.begin(115200);
  
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  
  delay(100);
  ld2410.begin(baudRates[currentBaudIdx], SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(500);
  
  drawInterface();
  
  startTime = millis();
}

void setBaudRate(int idx) {
  currentBaudIdx = max(0, min(idx, numBaudRates - 1));
  ld2410.end();
  delay(100);
  ld2410.begin(baudRates[currentBaudIdx], SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(500);
  Serial.print("Switched to baud: ");
  Serial.println(baudRates[currentBaudIdx]);
}

void drawInterface() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.print("LD2410C Interactive Scanner");
  M5.Lcd.drawLine(0, 20, 280, 20, WHITE);
  
  M5.Lcd.setCursor(5, 25);
  M5.Lcd.printf("W/Q or Up/Down: Change baud");
  M5.Lcd.setCursor(5, 40);
  M5.Lcd.print("Space: Clear counters");
  M5.Lcd.setCursor(5, 55);
  M5.Lcd.print("ESC: Exit test mode");
  M5.Lcd.drawFastHLine(0, 65, 280, BROWN);
  
  M5.Lcd.setCursor(5, 75);
  M5.Lcd.printf("Current Baud: %d", baudRates[currentBaudIdx]);
  
  M5.Lcd.setCursor(5, 95);
  M5.Lcd.print("Bytes Received: ");
  M5.Lcd.setCursor(5, 115);
  M5.Lcd.print("Status: ");
  
  M5.Lcd.fillRect(5, 250, 20, 15, GREEN);
  M5.Lcd.setCursor(30, 250);
  M5.Lcd.print("Active");
  M5.Lcd.fillRect(60, 250, 20, 15, RED);
  M5.Lcd.setCursor(90, 250);
  M5.Lcd.print("Idle");
}

//Use keysState() + iterate over word array (correct API!)
void checkKeyboard() {
  if (millis() - lastKeyTime < KEY_DEBOUNCE_MS) return;
  
  // Get keyboard state - returns KeysState struct
  auto& kbState = M5Cardputer.Keyboard.keysState();
  
  // Check modifier keys first
  if (kbState.shift || kbState.ctrl || kbState.alt) {
    return;  // Ignore modifier-only presses
  }
  
  // Iterate through pressed keys in the 'word' array
  for (auto k : kbState.word) {
    char key = k;
    
    Serial.printf("[KEY] 0x%02X '%c'\n", key, key);
    
    switch(key) {
      case 'w':
      case 'W':
        setBaudRate(currentBaudIdx + 1);
        drawInterface();
        lastKeyTime = millis();
        break;
        
      case 'q':
      case 'Q':
        setBaudRate(currentBaudIdx - 1);
        drawInterface();
        lastKeyTime = millis();
        break;
        
      case '\x1B':  // ESC (escape key ASCII code)
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(50, 100);
        M5.Lcd.print("Restarting...");
        delay(1000);
        esp_restart();
        break;
        
      case ' ':  // Spacebar - reset counters
        byteCount = 0;
        anyDataReceived = false;
        lastKeyTime = millis();
        break;
    }
  }
  
  kbState.fn = false;  // Reset function flag if needed
}

void loop() {
  M5.update();
  checkKeyboard();
  
  // Read incoming data from sensor
  while (ld2410.available()) {
    byte b = ld2410.read();
    anyDataReceived = true;
    byteCount++;
    
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(180, 75);
    M5.Lcd.printf("Last: 0x%02X", b);
    
    // Dump to serial as well
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    
    if (byteCount % 24 == 0) Serial.println("");
  }
  
  // Update stats display
  unsigned long elapsed = (millis() - startTime) / 1000;
  M5.Lcd.setCursor(150, 95);
  M5.Lcd.printf("%d", byteCount);
  
  M5.Lcd.setCursor(100, 75);
  M5.Lcd.printf("Elapsed: %ds", elapsed);
  
  // Status indicator
  M5.Lcd.setCursor(70, 115);
  if (anyDataReceived) {
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.print("ACTIVE");
  } else if (byteCount > 0 && byteCount < 10) {
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.print("LOW ACTIVITY");
  } else {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("IDLE");
  }
  
  delay(100);
}