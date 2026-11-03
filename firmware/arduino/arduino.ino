#include <Wire.h>

const int FSR_PINS[4] = {A3, A1, A2, A0};
const int LED_PINS[4] = {3, 6, 9, 12};
const int REST_VALUE = 600;
const int MPU_ADDR = 0x68;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  for (int i = 0; i < 4; i++) pinMode(LED_PINS[i], OUTPUT);
}

int readAccel() {
  return (Wire.read() << 8) | Wire.read();
}

void loop() {
  for (int i = 0; i < 4; i++) {
    int value = analogRead(FSR_PINS[i]);
    analogWrite(LED_PINS[i], map(constrain(value, REST_VALUE, 1023), REST_VALUE, 1023, 0, 255));
    Serial.print(value);
    Serial.print(", ");
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  Serial.print(readAccel());
  Serial.print(", ");
  Serial.print(readAccel());
  Serial.print(", ");
  Serial.println(readAccel());

  delay(20);
}
