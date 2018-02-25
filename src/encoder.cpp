#include "encoder.hpp"
#include "main.h"
#include "API.h"

float offset=0;
double inchesGet(){
  int ticks = encoderGet(driveEnc);
  const float diameter = 2.75;
  const float pi = 3.14159;
  double inches = pi*diameter*(double)ticks/360.0;
  return inches + offset;
}

void encZero(float newCurrent){
  encoderReset(driveEnc);
  offset = newCurrent;
  delay(20);
}

int armOffset=292;

double degreesGet(){
  int ticks = encoderGet(armEnc);
  double ratio = 1.0;
  double deg = ratio*(double)ticks + (double)armOffset;
  return deg;
}

void encOffset(int offsetDeg){
  armOffset = offsetDeg;
}
