#include "BluetoothManager.h"
#include <M5Cardputer.h>

void setup() {
  Serial.begin(115200);

  auto cfg = M5.config();

// Initialize Bluetooth
  bleManager = new BluetoothManager("M5-mmWave");
  bleManager->begin();

  //TODO: connect to sender (M5Dial)

M5Canvas canvas(&M5Cardputer.Display);
  M5Cardputer.Display.setRotation(1);  // 90° CW - right-side up
  
   uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);
}

void drawInterface() {
  canvas.fillSprite(BLACK);
  
  //display text "call incoming" when presence message received from bluetooth sender

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
}

void loop() {
  M5Cardputer.update();

}