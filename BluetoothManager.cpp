#include "BluetoothManager.h"

BluetoothManager::BluetoothManager(const char* name) : deviceName(name) {
}

bool BluetoothManager::begin() {
  Serial.println("[BLE] Initializing Bluetooth...");

  // Initialize BLE device
  BLEDevice::init(deviceName);

  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(this);

  // Create BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  pCharPresence = pService->createCharacteristic(
      CHAR_PRESENCE_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pCharPresence->addDescriptor(new BLE2902());

  pCharCommand = pService->createCharacteristic(
      CHAR_COMMAND_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pCharCommand->setCallbacks(this);

  pCharStatus = pService->createCharacteristic(
      CHAR_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharStatus->addDescriptor(new BLE2902());

  // Start service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started");
  return true;
}

void BluetoothManager::sendPresenceDetected(int distanceCm, int confidence) {
  if (!deviceConnected) return;

  String message = "PRESENCE_DETECTED:" + String(distanceCm) + "cm:" + String(confidence) + "%";
  pCharPresence->setValue(message.c_str());
  pCharPresence->notify();

  Serial.printf("[BLE] Sent: %s\n", message.c_str());
}

void BluetoothManager::sendPresenceCleared() {
  if (!deviceConnected) return;

  pCharPresence->setValue("PRESENCE_CLEARED");
  pCharPresence->notify();

  Serial.println("[BLE] Sent: PRESENCE_CLEARED");
}

void BluetoothManager::sendStatusUpdate(const char* status) {
  if (!deviceConnected) return;

  pCharStatus->setValue(status);
  pCharStatus->notify();

  Serial.printf("[BLE] Status: %s\n", status);
}

void BluetoothManager::onConnect(BLEServer* pServer) {
  deviceConnected = true;
  Serial.println("[BLE] Client connected");
}

void BluetoothManager::onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  Serial.println("[BLE] Client disconnected");
  BLEDevice::startAdvertising();
}

void BluetoothManager::onWrite(BLECharacteristic* pCharacteristic) {
  if (pCharacteristic->getUUID().toString() == CHAR_COMMAND_UUID) {
    const auto value = pCharacteristic->getValue();
    lastReceivedCommand = String(value.c_str());
    newCommandReceived = true;

    Serial.printf("[BLE] Received command: %s\n", lastReceivedCommand.c_str());
  }
}