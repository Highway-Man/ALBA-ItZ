#include "encoder.hpp"
#include "main.h"

double inchesGet(){
  int ticks = encoderGet(driveEnc);
  const float diameter = 2.75;
  const float pi = 3.14159;
  double inches = pi*diameter*(double)ticks/360.0;
  return inches;
}
