#include <M5Cardputer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

// Match these to BluetoothManager.h on the Dial
static const char* TARGET_DEVICE_NAME = "M5-mmWave";
static BLEUUID SERVICE_UUID("12345678-1234-5678-1234-56789abcdef0");
static BLEUUID CHAR_PRESENCE_UUID("12345678-1234-5678-1234-56789abcdef1");

// UI
M5Canvas canvas(&M5Cardputer.Display);

// BLE client state
static bool doConnect = false;
static bool connected = false;
static bool doScan = true;
static BLERemoteCharacteristic* pRemoteCharPresence = nullptr;
static BLEAdvertisedDevice* targetAdvertisedDevice = nullptr;
static BLEClient*  pClient = nullptr;

// Call incoming UI state
static volatile bool callIncoming = false;
static unsigned long callIncomingSince = 0;
const unsigned long CALL_TIMEOUT_MS = 15000; // auto-clear after 15s

void tonePlaceholder(unsigned int freq, unsigned long duration) {
  // Placeholder for buzzer/speaker. Implement with hardware-specific API later.
}

// Notification callback (called from BLE stack context)
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  String s;
  for (size_t i=0;i<length;i++) s += (char)pData[i];

  Serial.printf("[BLE CLIENT] Notif: %s\n", s.c_str());

  if (s.startsWith("PRESENCE_DETECTED")) {
    // Show UI and (optionally) play tone
    callIncoming = true;
    callIncomingSince = millis();
    // tone can be implemented later; for now we just note it
    // tonePlaceholder(2000, 200);
  } else if (s.startsWith("PRESENCE_CLEARED")) {
    callIncoming = false;
  }
}

// Advertised device callback: detects target device by name or service UUID
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Found device: %s\n", advertisedDevice.toString().c_str());
    // Check name
    if (advertisedDevice.haveName() && advertisedDevice.getName() == TARGET_DEVICE_NAME) {
      Serial.println("-> Target by name");
      targetAdvertisedDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
      BLEDevice::getScan()->stop();
    } else if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      Serial.println("-> Target by service UUID");
      targetAdvertisedDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
      BLEDevice::getScan()->stop();
    }
  }
};

bool connectToServer() {
  if (!targetAdvertisedDevice) return false;

  Serial.print("[BLE CLIENT] Forming a connection to ");
  Serial.println(targetAdvertisedDevice->getAddress().toString().c_str());

  pClient = BLEDevice::createClient();
  Serial.println("[BLE CLIENT] - Created client");

  if (!pClient->connect(targetAdvertisedDevice)) {
    Serial.println("[BLE CLIENT] - Failed to connect");
    return false;
  }

  Serial.println("[BLE CLIENT] - Connected to server");
  BLERemoteService* pRemoteService = nullptr;
  try {
    pRemoteService = pClient->getService(SERVICE_UUID);
  } catch (...) { pRemoteService = nullptr; }

  if (pRemoteService == nullptr) {
    Serial.println("[BLE CLIENT] - Failed to find service.");
    pClient->disconnect();
    return false;
  }

  pRemoteCharPresence = pRemoteService->getCharacteristic(CHAR_PRESENCE_UUID);
  if (pRemoteCharPresence == nullptr) {
    Serial.println("[BLE CLIENT] - Failed to find presence characteristic.");
    pClient->disconnect();
    return false;
  }

  if(pRemoteCharPresence->canNotify()) {
    pRemoteCharPresence->registerForNotify(notifyCallback);
    Serial.println("[BLE CLIENT] - Registered for presence notifications");
  }

  connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Cardputer BLE Client ===");

  // Cardputer init
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);  // 90° CW
  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);

  // BLE init
  BLEDevice::init(""); // empty local name (client)
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);

  Serial.println("[BLE CLIENT] Setup complete, starting loop...");
}

void drawInterface() {
  canvas.fillSprite(BLACK);

  canvas.setTextSize(1);
  canvas.setCursor(5, 5);
  canvas.setTextColor(WHITE);
  canvas.println("Cardputer BLE Client");

  // BLE status
  canvas.setCursor(5, 24);
  if (connected) {
    canvas.setTextColor(GREEN);
    canvas.printf("BLE: Connected");
  } else {
    canvas.setTextColor(YELLOW);
    canvas.printf("BLE: Waiting...");
  }

  // Call incoming
  canvas.setTextSize(2);
  canvas.setCursor(5, 48);
  if (callIncoming) {
    canvas.setTextColor(RED);
    canvas.printf("Call incoming");
  } else {
    canvas.setTextColor(LIGHTGREY);
    canvas.printf("Waiting...");
  }

  canvas.pushSprite(2, 2);
}

void loop() {
  M5Cardputer.update();

  // manage BLE connection
  if (!connected) {
    if (doScan) {
      Serial.println("[BLE CLIENT] Scanning for target...");
      BLEDevice::getScan()->start(3);
      //commented out due to undeclared foundDevices. 
   //   Serial.printf("[BLE CLIENT] Scan complete, %d devices found\n", foundDevices.getCount());
      // If device was found via callbacks, doConnect will be set
    }

    if (doConnect) {
      Serial.println("[BLE CLIENT] Attempting connect...");
      if (connectToServer()) {
        Serial.println("[BLE CLIENT] Connected and subscribed");
      } else {
        Serial.println("[BLE CLIENT] Connect failed; will retry");
        // cleanup
        if (pClient) {
          pClient->disconnect();
          delete pClient;
          pClient = nullptr;
        }
        connected = false;
        doScan = true;
        doConnect = false;
        // free target advert device so next scan picks it up fresh
        if (targetAdvertisedDevice) {
          delete targetAdvertisedDevice;
          targetAdvertisedDevice = nullptr;
        }
        delay(1000);
      }
    }
  } else {
    // if still connected, check timeout/auto-clear UI
    if (callIncoming && millis() - callIncomingSince > CALL_TIMEOUT_MS) {
      callIncoming = false;
    }
    // check if still connected at the BLE client level
    if (pClient && !pClient->isConnected()) {
      Serial.println("[BLE CLIENT] Lost connection");
      connected = false;
      callIncoming = false;
      // cleanup and re-scan
      if (pClient) {
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
      }
      doScan = true;
      doConnect = false;
      if (targetAdvertisedDevice) { delete targetAdvertisedDevice; targetAdvertisedDevice = nullptr; }
    }
  }

  drawInterface();
  delay(50);
}
