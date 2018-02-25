#include "API.h"
#include "PID.hpp"
#include "main.h"
#include "math.h"
#include "subsystems.h"

int delayTime;
int pTime, t, dt, slewRate, controlLast;

void Pid::init(float P, float I, float D, short dMax, int dT, double (*sensor)(void), void (*motors)(short)){
  kP = P;
  kD = D;
  kI = I;
  input = sensor;
  pError=0.0;
  pTime = millis();
  integral = 0.0;
  output = motors;
  deltaMax = dMax;
  delayTime=dT;
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
  int i=0, dir=sign(control);
  while(fabs(error) > thresh){
    calc();
    set(control);
    if(i>10){
      //printf("%f, %f, %d\n", error, derivative, control);
      i=0;
    }
    delay(delayTime);
    i++;
  }
  output(dir*10);
}

void Pid::moveToUntil(float tar, float thresh, int ms){
  long timeElapsed, timeStarted;
  target = tar;
  calc();
  timeStarted = millis();
  timeElapsed = millis() - timeStarted;
  delay(20);
  while((fabs(error) > thresh || fabs(velocity) > 0.001) && timeElapsed < ms){
    calc();
    set(control);
    timeElapsed = millis() - timeStarted;
    printf("%f, \n", position);
    delay(20);
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
    set(speed);
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
    set(speed);
    timeElapsed = millis() - timeStarted;
    delay(delayTime);
  }
  set(2*sign(speed));
}

void Pid::stack(int height, int ret){
  clawSet(10);
  int prev = target;
  if (height == 1)
    target = 280;//840
  else if (height == 2)
    target = 265;//800
  else if (height == 3)
    target = 250;//775
  else if (height == 4)
    target = 240;//745
  else if (height == 5)
    target = 230;//700
  else if (height == 6)
    target = 220;//640
  else if (height == 7)
    target = 212;//600
  else if (height == 8)
    target = 200;//600
  calc();
  //890, 850, 810, 785, 765, 750, 733, 670,
  while(fabs(error > 10)){
    calc();
    set(control);
    delay(delayTime);
  }
  clawSet(-127);
  for(int i=0; i<12;i++){
    calc();
    set(control);
    delay(delayTime);
  }
  if(ret == 1){
      target=prev;
    calc();
    while (fabs(error) > 90) {
      calc();
      set(control);
      delay(delayTime);
    }
    clawSet(127);
    moveBeyond(target, 10);
  }
  else{
    target = 190;
    moveToUntil(target, 20, 1000);
    clawSet(0);
    chainbarSet(-10);
  }
}
