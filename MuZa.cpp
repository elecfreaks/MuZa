#include "MuZa.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

#define LIGHT_SENSOR_PIN  A6
#define NOISE_SENSOR_PIN  A7
#define BUZZER_PIN        12
#define RGB_PIN           13
#define S1_PIN            9
#define S2_PIN            10
#define M1_PIN_P          11
#define M1_PIN_N          3
#define M2_PIN_P          5
#define M2_PIN_N          6

#define SERVO_MAX_ANGLE   335

Adafruit_NeoPixel pixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
Servo servo1;
Servo servo2;

MuZa::MuZa()
{
  _tempo = 60;
}

void MuZa::begin()
{
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(M1_PIN_P, OUTPUT);
  pinMode(M1_PIN_N, OUTPUT);
  pinMode(M2_PIN_P, OUTPUT);
  pinMode(M2_PIN_N, OUTPUT);
  pixel.begin();
  servo1.attach(S1_PIN);
  servo2.attach(S2_PIN);
}

int MuZa::getLightSensor()
{
  return analogRead(LIGHT_SENSOR_PIN);
}

int MuZa::getNoiseSensor()
{
  return analogRead(NOISE_SENSOR_PIN);
}

void MuZa::tone(uint16_t frequency, uint32_t duration)
{
  if (frequency == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
    return;
  }

  int period = 1000000L / frequency;
  int pulse = period / 2;
  for (uint32_t i = 0; i < duration * 1000L; i += period)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(pulse);
  }
}

/**
 * 20~500
 */
void MuZa::setTempo(uint16_t tempo)
{
  if (tempo < 20) {
    tempo = 20;
  } else if (tempo > 500) {
    tempo = 500;
  }
  _tempo = tempo;
}

uint32_t MuZa::beatsToMS(float beats)
{
  return (60.0 / _tempo) * beats * 1000;
}

void MuZa::showRGB(uint8_t r, uint8_t g, uint8_t b)
{
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

void MuZa::clearRGB()
{
  pixel.clear();
  pixel.show();
}

void MuZa::setRGBBrightness(uint8_t brightness)
{
  pixel.setBrightness(brightness);
}

void MuZa::setServoAngle(ServoID id, int angle)
{
  angle = map(angle, 0, SERVO_MAX_ANGLE, 0, 180);

  switch (id)
  {
  case S1:
    servo1.write(angle);
    break;
  case S2:
    servo2.write(angle);
    break;
  }
}

void MuZa::setMotorSpeed(MotorID id, int percent)
{
  int val = (int)(abs(percent) / 100.0 * 255);

  switch (id)
  {
  case M1:
    if (percent > 0) {
      setDualPWM(M1_PIN_P, M1_PIN_N, 0, val);
    } else {
      setDualPWM(M1_PIN_P, M1_PIN_N, val, 0);
    }
    break;
  case M2:
    if (percent > 0) {
      setDualPWM(M2_PIN_P, M2_PIN_N, 0, val);
    } else {
      setDualPWM(M2_PIN_P, M2_PIN_N, val, 0);
    }
    break;
  }
}

void MuZa::brakeMotor(MotorID id)
{
  switch (id)
  {
  case M1:
    setDualPWM(M1_PIN_P, M1_PIN_N, 255, 255);
    break;
  case M2:
    setDualPWM(M2_PIN_P, M2_PIN_N, 255, 255);
    break;
  }
}

void MuZa::setDualPWM(uint8_t pin1, uint8_t pin2, int val1, int val2)
{
  analogWrite(pin1, val1);
  analogWrite(pin2, val2);
}
