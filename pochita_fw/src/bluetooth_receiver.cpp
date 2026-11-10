#include "bluetooth_receiver.h"

#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

namespace bluetooth_receiver {
namespace {
constexpr char SERVICE_UUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr char SENDER_NAME[] = "Makima";

BLEClient* client = nullptr;
BLERemoteCharacteristic* dataCharacteristic = nullptr;
BLEAddress senderAddress("00:00:00:00:00:00");
volatile bool senderFound = false;
volatile bool packetReady = false;
volatile ControlPacket latestPacket{};
unsigned long nextScanTime = 0;

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    Serial.print(F("Found BLE device: "));
    if (advertisedDevice.haveName()) {
      Serial.print(advertisedDevice.getName().c_str());
    } else {
      Serial.print(F("<no name>"));
    }
    Serial.print(F(" ["));
    Serial.print(advertisedDevice.getAddress().toString().c_str());
    Serial.println(']');

    if (advertisedDevice.haveName() &&
        advertisedDevice.getName() == SENDER_NAME) {
      senderAddress = advertisedDevice.getAddress();
      senderFound = true;
      Serial.println(F("Makima found"));
    }
  }
};

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient*) override {
    Serial.println(F("Connected to Makima"));
  }

  void onDisconnect(BLEClient*) override {
    dataCharacteristic = nullptr;
    nextScanTime = millis() + 1000;
    Serial.println(F("BLE disconnected"));
  }
};

void onNotification(BLERemoteCharacteristic*, uint8_t* data, size_t length,
                    bool) {
  if (length != sizeof(ControlPacket)) {
    Serial.print(F("Received packet with unexpected length: "));
    Serial.println(length);
    return;
  }

  memcpy((void*)&latestPacket, data, sizeof(ControlPacket));
  packetReady = true;
}

void scanForSender() {
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), true);
  scan->setActiveScan(true);
  scan->start(5, false);
  scan->clearResults();
}

bool connectToSender() {
  if (!senderFound) {
    Serial.println(F("Makima not found in scan"));
    return false;
  }

  Serial.println(F("Connecting to Makima..."));
  client = BLEDevice::createClient();
  client->setClientCallbacks(new ClientCallbacks());
  if (!client->connect(senderAddress)) {
    Serial.println(F("BLE connection failed"));
    delete client;
    client = nullptr;
    senderFound = false;
    return false;
  }

  BLERemoteService* service = client->getService(SERVICE_UUID);
  if (service == nullptr) {
    Serial.println(F("Service not found"));
    client->disconnect();
    delete client;
    client = nullptr;
    senderFound = false;
    return false;
  }

  dataCharacteristic = service->getCharacteristic(CHARACTERISTIC_UUID);
  if (dataCharacteristic == nullptr || !dataCharacteristic->canNotify()) {
    Serial.println(F("Notify characteristic not found"));
    client->disconnect();
    delete client;
    client = nullptr;
    dataCharacteristic = nullptr;
    senderFound = false;
    return false;
  }

  dataCharacteristic->registerForNotify(onNotification);
  Serial.println(F("Subscribed to packet notifications"));
  return true;
}
}  // namespace

void begin() {
  BLEDevice::init("Pochita");
  scanForSender();
}

void update() {
  if (client != nullptr && client->isConnected()) {
    return;
  }

  if (client != nullptr) {
    delete client;
    client = nullptr;
    dataCharacteristic = nullptr;
  }

  if (millis() < nextScanTime) {
    return;
  }

  senderFound = false;
  scanForSender();
  if (!connectToSender()) {
    nextScanTime = millis() + 1000;
  }
}

bool isConnected() {
  return client != nullptr && client->isConnected();
}

bool takePacket(ControlPacket& packet) {
  if (!packetReady) {
    return false;
  }

  noInterrupts();
  memcpy(&packet, (const void*)&latestPacket, sizeof(packet));
  packetReady = false;
  interrupts();
  return true;
}

}  // namespace bluetooth_receiver
