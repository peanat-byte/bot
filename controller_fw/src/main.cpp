#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>

#define L_JOYSTICK_H 2 // OUT1
#define L_JOYSTICK_V 4 // OUT2
#define R_JOYSTICK_H 32 // OUT1
#define R_JOYSTICK_V 33 // OUT2

#define ARMED 5
#define BUTTON 16

namespace {
constexpr uint8_t i2cSdaPin = 21;
constexpr uint8_t i2cSclPin = 22;
constexpr uint8_t oledAddress = 0x3C;
constexpr uint8_t oledWidth = 128;
constexpr uint8_t oledHeight = 64;
constexpr int joystickMaximum = 4000;

struct __attribute__((packed)) ControlPacket {
  uint16_t leftHorizontal;
  uint16_t leftVertical;
  uint16_t rightHorizontal;
  uint16_t rightVertical;
  uint8_t switches;
};

static_assert(sizeof(ControlPacket) == 9, "Unexpected control packet size");

Adafruit_SSD1306 display(oledWidth, oledHeight, &Wire, -1);
BLECharacteristic* dataCharacteristic;
bool bleConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
  }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    server->startAdvertising();
  }
};

void drawJoystick(int x, int y, const __FlashStringHelper* label, int horizontal, int vertical) {
  constexpr int boxWidth = 58;
  constexpr int boxHeight = 43;
  constexpr int graphXInset = 8;
  constexpr int graphYInset = 16;
  constexpr int graphWidth = 42;
  constexpr int graphHeight = 22;

  display.setCursor(x + 2, y + 2);
  display.print(label);
  display.drawRect(x, y, boxWidth, boxHeight, SSD1306_WHITE);

  const int graphX = x + graphXInset;
  const int graphY = y + graphYInset;
  const int centerX = graphX + graphWidth / 2;
  const int centerY = graphY + graphHeight / 2;
  const int pointX = map(constrain(horizontal, 0, joystickMaximum), 0, joystickMaximum, graphX, graphX + graphWidth - 1);
  const int pointY = map(constrain(vertical, 0, joystickMaximum), 0, joystickMaximum, graphY + graphHeight - 1, graphY);

  display.drawLine(centerX, graphY, centerX, graphY + graphHeight - 1, SSD1306_WHITE);
  display.drawLine(graphX, centerY, graphX + graphWidth - 1, centerY, SSD1306_WHITE);
  display.fillCircle(pointX, pointY, 3, SSD1306_WHITE);
}
}

void setup() {
  Serial.begin(115200);

  BLEDevice::init("Pochitaa");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(
      "4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  dataCharacteristic = service->createCharacteristic(
      "beb5483e-36e1-4688-b7f5-ea07361b26a8",
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  dataCharacteristic->addDescriptor(new BLE2902());
  service->start();
  server->getAdvertising()->start();
  
  analogReadResolution(12);
  pinMode(BUTTON, INPUT);
  pinMode(ARMED, INPUT);

  Wire.begin(i2cSdaPin, i2cSclPin);
  if (!display.begin(SSD1306_SWITCHCAPVCC, oledAddress)) {
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.display();
}

void loop() {
  const int leftHorizontal = analogRead(L_JOYSTICK_H);
  const int leftVertical = analogRead(L_JOYSTICK_V);
  const int rightHorizontal = analogRead(R_JOYSTICK_H);
  const int rightVertical = analogRead(R_JOYSTICK_V);
  const int arm1 = digitalRead(ARMED);
  constexpr int arm2 = 0;
  const int button1 = digitalRead(BUTTON);
  constexpr int button2 = 0;
  constexpr int button3 = 0;

  ControlPacket packet{
      static_cast<uint16_t>(leftHorizontal),
      static_cast<uint16_t>(leftVertical),
      static_cast<uint16_t>(rightHorizontal),
      static_cast<uint16_t>(rightVertical),
      static_cast<uint8_t>((arm1 ? 1 : 0) |
                          (arm2 ? 1 << 1 : 0) |
                          (button1 ? 1 << 2 : 0) |
                          (button2 ? 1 << 3 : 0) |
                          (button3 ? 1 << 4 : 0))};
  if (bleConnected) {
    dataCharacteristic->setValue(reinterpret_cast<uint8_t*>(&packet),
                                  sizeof(packet));
    dataCharacteristic->notify();
  }

  Serial.print(arm1);
  Serial.print(',');
  Serial.print(arm2);
  Serial.print(',');
  Serial.print(leftHorizontal);
  Serial.print(',');
  Serial.print(leftVertical);
  Serial.print(',');
  Serial.print(rightHorizontal);
  Serial.print(',');
  Serial.print(rightVertical);
  Serial.print(',');
  Serial.print(button1);
  Serial.print(',');
  Serial.print(button2);
  Serial.print(',');
  Serial.println(button3);

  // Serial.print(F("Left H: "));
  // Serial.print(leftHorizontal);
  // Serial.print(F(" | Left V: "));
  // Serial.print(leftVertical);
  // Serial.print(F(" | Right H: "));
  // Serial.print(rightHorizontal);
  // Serial.print(F(" | Right V: "));
  // Serial.println(rightVertical);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(digitalRead(ARMED) ? F("ARMED") : F("DISARMED"));
  drawJoystick(3, 10, F("LEFT"), leftHorizontal, leftVertical);
  drawJoystick(67, 10, F("RIGHT"), rightHorizontal, rightVertical);
  display.setCursor(0, 56);
  display.print(F("BUTTON "));
  display.print(digitalRead(BUTTON) ? F(":(") : F(":)"));
  display.display();

  delay(100);
}