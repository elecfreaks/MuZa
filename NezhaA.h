#ifndef _NEZHAA_H_
#define _NEZHAA_H_

#include <Arduino.h>

enum ServoID {
  S1,
  S2
};

enum MotorID {
  M1,
  M2
};

class NezhaA
{
public:
  NezhaA();
  void begin();
  int getLightSensor();
  int getNoiseSensor();
  void tone(uint16_t frequency, uint32_t duration);
  void setTempo(uint16_t tempo);
  uint32_t beatsToMS(float beats);
  void showRGB(uint8_t r, uint8_t g, uint8_t b);
  void clearRGB();
  void setRGBBrightness(uint8_t brightness);
  void setServoAngle(ServoID id, int angle);
  void setMotorSpeed(MotorID id, int percent);
  void brakeMotor(MotorID id);

private:
  uint16_t _tempo;
  void setDualPWM(uint8_t pin1, uint8_t pin2, int val1, int val2);
};

#endif
