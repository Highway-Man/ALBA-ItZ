#include "API.h"
#include "sonar.hpp"

const double vPin = 5.0/512.0;
const double anPv = 4095.0/5.0;
long us::read(){
  analog = analogRead(port);
  return analog;
}
double us::calculate(){
  read();
  voltage = analog / anPv;
  inches = voltage / vPin;
  return inches;
}

float us::get(){
  calculate();
  return inches;
}

void us::init(short pin){
  port = pin;
}
