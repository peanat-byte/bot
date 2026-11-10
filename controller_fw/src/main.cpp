#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>

namespace {
constexpr uint8_t L_JOYSTICK_H_PIN = 2; // OUT1
constexpr uint8_t L_JOYSTICK_V_PIN = 4; // OUT2
constexpr uint8_t R_JOYSTICK_H_PIN = 32; // OUT1
constexpr uint8_t R_JOYSTICK_V_PIN = 33; // OUT2

constexpr uint8_t ARMED_PIN = 5;
constexpr uint8_t BUTTON_PIN = 16;

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int JOYSTICK_MAX = 4000;

struct __attribute__((packed)) ControlPacket {
  uint16_t leftHorizontal;
  uint16_t leftVertical;
  uint16_t rightHorizontal;
  uint16_t rightVertical;
  uint8_t switches;
};

static_assert(sizeof(ControlPacket) == 9, "Unexpected control packet size");

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
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
  const int pointX = map(constrain(horizontal, 0, JOYSTICK_MAX), 0, JOYSTICK_MAX, graphX, graphX + graphWidth - 1);
  const int pointY = map(constrain(vertical, 0, JOYSTICK_MAX), 0, JOYSTICK_MAX, graphY + graphHeight - 1, graphY);

  display.drawLine(centerX, graphY, centerX, graphY + graphHeight - 1, SSD1306_WHITE);
  display.drawLine(graphX, centerY, graphX + graphWidth - 1, centerY, SSD1306_WHITE);
  display.fillCircle(pointX, pointY, 3, SSD1306_WHITE);
}
}

void setup() {
  Serial.begin(115200);

  BLEDevice::init("Makima");
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
  pinMode(BUTTON_PIN, INPUT);
  pinMode(ARMED_PIN, INPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.display();
}

void loop() {
  const int leftHorizontal = analogRead(L_JOYSTICK_H_PIN);
  const int leftVertical = analogRead(L_JOYSTICK_V_PIN);
  const int rightHorizontal = analogRead(R_JOYSTICK_H_PIN);
  const int rightVertical = analogRead(R_JOYSTICK_V_PIN);
  const int arm1 = digitalRead(ARMED_PIN);
  constexpr int arm2 = 0;
  const int button1 = digitalRead(BUTTON_PIN);
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

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(digitalRead(ARMED_PIN) ? F("ARMED") : F("DISARMED"));
  drawJoystick(3, 10, F("LEFT"), leftHorizontal, leftVertical);
  drawJoystick(67, 10, F("RIGHT"), rightHorizontal, rightVertical);
  display.setCursor(0, 56);
  display.print(F("BUTTON "));
  display.print(digitalRead(BUTTON_PIN) ? F(":(") : F(":)"));
  display.display();

  delay(100);
}