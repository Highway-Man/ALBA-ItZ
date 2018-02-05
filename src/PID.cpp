#include "API.h"
#include "PID.hpp"
#include "main.h"
#include "math.h"

void Pid::init(float P, float I, float D, short dMax, double (*sensor)(void), void (*motors)(short)){
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
  if(error != pError || dt > 100){
    derivative = (error - pError)/dt;
    velocity = derivative*1000;

    integral += error*dt;

    pTime = t;
    pError = error;
  }

  control = kP*error + kD*derivative;// + kI*integral;
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
  int i=0;
  while(fabs(error) > thresh || fabs(velocity) > 0.001){
    calc();
    set(control);
    if(i>4){
      //printf("%f, %f, %d\n", error, derivative, control);
      i=0;
    }
    delay(delayTime);
    i++;
  }
}
void Pid::moveBeyond(float tar, float thresh){
  target = tar;
  calc();
  int i=0;
  while(fabs(error) > thresh){
    calc();
    set(control);
    if(i>10){
      printf("%f, %f, %d\n", error, derivative, control);
      i=0;
    }
    delay(delayTime);
    i++;
  }
}

void Pid::moveToUntil(float tar, float thresh, int ms){
  long timeElapsed, timeStarted;
  target = tar;
  calc();
  timeStarted = millis();
  timeElapsed = millis() - timeStarted;
  while((fabs(error) > thresh || fabs(velocity) > 0.001) && timeElapsed < ms){
    calc();
    set(control);
    timeElapsed = millis() - timeStarted;
    delay(delayTime);
  }
}

void Pid::moveFor(int speed, int ms){
  long timeElapsed, timeStarted;
  set(speed);
  delay(200);
  timeStarted = millis();
  timeElapsed = millis() - timeStarted;
  calc();
  while(timeElapsed < ms && fabs(velocity) > 0.001){
    calc();
    timeElapsed = millis() - timeStarted;
    delay(delayTime);
  }
  set(2*sign(speed));
}

void Pid::moveWhile(int speed, float tar, float thresh, int ms){
  long timeElapsed, timeStarted;
  set(speed);
  target = tar;
  delay(100);
  timeStarted = millis();
  timeElapsed = millis() - timeStarted;
  calc();
  while(timeElapsed < ms && fabs(error) > thresh){
    calc();
    timeElapsed = millis() - timeStarted;
    delay(delayTime);
  }
  set(2*sign(speed));
}
