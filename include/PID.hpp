#ifndef _PID_H
#define _PID_H

class Pid{
private:
double (*input)(void);
void (*output)(short);
public:
  const unsigned short lin[128] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 88, 89, 89, 90, 90,127,127,127
};
  float kP, kD, kI;
  float error, integral, derivative, pError;
  float target;
  double position, velocity;
  short deltaMax, control;
  void init(float P, float D, float I, short dMax, int dT, double (*sensor)(void), void (*motors)(short));
  void calc(void);
  void set(short value);
  void moveTo(float tar, float thresh);
  void moveBeyond(float tar, float thresh);
  void moveToUntil(float tar, float thresh, int ms);
  void moveFor(int speed, int ms);
  void moveWhile(int speed, float tar, float thresh, int ms);
  void stack(int height, int ret);

};
extern Pid turn;
extern Pid drive;
extern Pid lift;


#endif
