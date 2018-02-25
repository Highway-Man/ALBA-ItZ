#include "gyro.hpp"
#include "main.h"

double gyroOffset;

double gyroRead(){
  return (float) -gyroGet(gyro) + gyroOffset;
}

void gyroZero(double newCurrent){
  gyroOffset = newCurrent;
  gyroReset(gyro);
}
