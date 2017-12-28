#ifndef SONAR_H
#define _SONAR_H

class us{
private:
  short port;
  long analog;
  double voltage, inches;
  long read(void);
  double calculate(void);
public:
  float get(void);
  void init(short pin);
};

#endif
