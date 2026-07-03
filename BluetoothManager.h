#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BluetoothManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
  // Constructor
  BluetoothManager(const char* deviceName = "M5-mmWave-Sensor");

  // Initialization
  bool begin();

  // Send presence detection
  void sendPresenceDetected(int distanceCm, int confidence);
  void sendPresenceCleared();
  void sendStatusUpdate(const char* status);

  // Getters
  bool isConnected() const { return deviceConnected; }
  const char* getLastCommand() const { return lastReceivedCommand.c_str(); }
  bool hasNewCommand() const { return newCommandReceived; }
  void clearNewCommandFlag() { newCommandReceived = false; }

private:
  // BLE objects
  BLEServer* pServer = nullptr;
  BLEService* pService = nullptr;
  BLECharacteristic* pCharPresence = nullptr;
  BLECharacteristic* pCharCommand = nullptr;
  BLECharacteristic* pCharStatus = nullptr;

  // State
  bool deviceConnected = false;
  bool newCommandReceived = false;
  String lastReceivedCommand = "";
  const char* deviceName;

  // UUIDs
  const char* SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";
  const char* CHAR_PRESENCE_UUID = "12345678-1234-5678-1234-56789abcdef1";  // Notify
  const char* CHAR_COMMAND_UUID = "12345678-1234-5678-1234-56789abcdef2";   // Write
  const char* CHAR_STATUS_UUID = "12345678-1234-5678-1234-56789abcdef3";    // Read/Notify

  // Callbacks
  void onConnect(BLEServer* pServer) override;
  void onDisconnect(BLEServer* pServer) override;
  void onWrite(BLECharacteristic* pCharacteristic) override;
};

#endif