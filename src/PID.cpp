#include "API.h"
#include "PID.hpp"
#include "main.h"

void Pid::init(float P, float D, float I, short dMax, double (*sensor)(void), void (*motors)(short)){
  kP = P;
  kD = D;
  kI = I;
  input = sensor;
  pError=0.0;
  pTime = millis();
  integral = 0.0;
  output = motors;
  deltaMax = dMax;
}
void Pid::calc(void){
  position = input();
  t = millis();
  error = target-position;

  dt = t-pTime;
  derivative = (error - pError)/dt;

  integral += error*dt;

  control = kP*error + kD*derivative + kI*integral;

  pTime = t;

}
void Pid::set(short value){
  slewRate = value - controlLast;
  if(abs(slewRate) > deltaMax)
    value = controlLast + sign(slewRate)*deltaMax;
  output(value);
  controlLast = value;
}
void Pid::moveTo(float tar, float thresh){
  target = tar;
  calc();
  while(error < thresh){
    calc();
    set(control);
    delay(delayTime);
  }
}
void Pid::moveToUntil(float tar, float thresh, int ms){
  long timeElapsed, timeStarted;
  target = tar;
  calc();
  timeStarted = millis();
  timeElapsed = millis() - timeStarted;
  while(error < thresh && timeElapsed < ms){
    calc();
    set(control);
    timeElapsed = millis() - timeStarted;
    delay(delayTime);
  }
}
